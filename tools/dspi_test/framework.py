"""
framework.py — test registry, the Check assertion helper, the serial Runner,
and report emission (console + Markdown + JSON).

Design notes:
  * The DSPi is a single shared hardware resource.  Tests run serially in
    registration order, never in parallel.
  * Each test is a function `fn(dev, profile, chk)` that records sub-checks on
    `chk`.  The test PASSes iff `chk` collected no failures.  Raising `Skip`
    marks SKIP; any other exception (incl. Disconnected) is ERROR/abort.
  * Liveness: after every test flagged `mutating=True`, the runner pings
    GET_PLATFORM.  A dead device turns the result into a hard failure and aborts
    the run (a crash must not be silently swallowed by later tests).
  * Flash budget: each test declares `flash` (erase-cycle cost).  Flash tests are
    skipped unless `--allow-flash`, and the runner enforces a hard cap.
"""

from __future__ import annotations

import json
import time
import traceback
from dataclasses import dataclass, field

from .device import DspiDevice, Disconnected, Stall


# --- Skip control flow ------------------------------------------------------

class Skip(Exception):
    """Raise inside a test to mark it SKIP with a reason."""


# --- The Check accumulator --------------------------------------------------

class Check:
    """Collects sub-assertions; the test fails iff any failed.

    Helpers return the actual value (or True/False) so tests can branch.
    """

    def __init__(self):
        self.failures: list[str] = []
        self.notes: list[str] = []

    def ok(self, cond: bool, msg: str) -> bool:
        if not cond:
            self.failures.append(msg)
        return bool(cond)

    @staticmethod
    def _short(v, limit=96):
        r = repr(v)
        return r if len(r) <= limit else r[:limit] + f"...<{len(r)} chars>"

    def eq(self, actual, expected, msg: str = "") -> bool:
        return self.ok(actual == expected,
                       f"{msg}: expected {self._short(expected)}, got {self._short(actual)}")

    def ne(self, actual, unexpected, msg: str = "") -> bool:
        return self.ok(actual != unexpected, f"{msg}: should not equal {self._short(unexpected)}")

    def approx(self, actual, expected, tol, msg: str = "") -> bool:
        try:
            good = abs(actual - expected) <= tol
        except TypeError:
            good = False
        return self.ok(good, f"{msg}: expected {expected}±{tol}, got {actual}")

    def in_range(self, actual, lo, hi, msg: str = "") -> bool:
        return self.ok(lo <= actual <= hi, f"{msg}: expected in [{lo},{hi}], got {actual}")

    def member(self, actual, allowed, msg: str = "") -> bool:
        return self.ok(actual in allowed, f"{msg}: expected one of {allowed}, got {actual!r}")

    def stalls(self, fn, msg: str = "") -> bool:
        """Assert `fn()` STALLs.  Returns True if it did."""
        try:
            r = fn()
        except Stall:
            return True
        self.failures.append(f"{msg}: expected STALL, got {r!r}")
        return False

    def no_stall(self, fn, msg: str = ""):
        """Assert `fn()` does NOT stall.  Returns its result, or None on stall."""
        try:
            return fn()
        except Stall as e:
            self.failures.append(f"{msg}: unexpected STALL ({e})")
            return None

    def note(self, msg: str):
        self.notes.append(msg)


# --- Test registry ----------------------------------------------------------

@dataclass
class TestCase:
    name: str
    group: str
    fn: object
    mutating: bool = False
    flash: int = 0           # erase-cycle cost
    needs: str | None = None  # capability gate: 'flash' | 'factory_reset' | None
    doc: str = ""


REGISTRY: list[TestCase] = []


def catalog_markdown(cases=None) -> str:
    """Generate the test catalog from registered metadata (never drifts from code)."""
    cases = cases if cases is not None else REGISTRY
    groups: dict[str, list] = {}
    for tc in cases:
        groups.setdefault(tc.group, []).append(tc)
    lines = ["# DSPi Control-Plane QA — Test Catalog", "",
             f"*{len(cases)} tests across {len(groups)} groups. Generated from test metadata.*", ""]
    for g in sorted(groups):
        lines += [f"## {g}", "", "| Test | Tags | What it checks |", "|---|---|---|"]
        for tc in groups[g]:
            tags = []
            if tc.flash:
                tags.append(f"flash×{tc.flash}")
            if tc.needs == "factory_reset":
                tags.append("factory-reset")
            if tc.mutating:
                tags.append("mutating")
            doc = (tc.doc or "").replace("|", "\\|")
            lines.append(f"| `{tc.name}` | {', '.join(tags) or '—'} | {doc} |")
        lines.append("")
    return "\n".join(lines)


def test(group: str, mutating: bool = False, flash: int = 0, needs: str | None = None):
    """Decorator registering a test function `fn(dev, profile, chk)`."""
    def deco(fn):
        REGISTRY.append(TestCase(
            name=fn.__name__, group=group, fn=fn,
            mutating=mutating, flash=flash,
            needs=needs if needs else ("flash" if flash else None),
            doc=(fn.__doc__ or "").strip().split("\n")[0]))
        return fn
    return deco


# --- Results & runner -------------------------------------------------------

@dataclass
class Result:
    name: str
    group: str
    status: str            # PASS | FAIL | SKIP | ERROR
    detail: str = ""
    notes: list = field(default_factory=list)
    duration_ms: float = 0.0


class Runner:
    DEFAULT_FLASH_CAP = 30

    def __init__(self, dev: DspiDevice, profile, *, allow_flash=False,
                 allow_factory_reset=False, flash_cap=None, verbose=True):
        self.dev = dev
        self.profile = profile
        self.allow_flash = allow_flash
        self.allow_factory_reset = allow_factory_reset
        self.flash_cap = flash_cap if flash_cap is not None else self.DEFAULT_FLASH_CAP
        self.verbose = verbose
        self.results: list[Result] = []
        self.flash_used = 0
        self.aborted = False
        self.abort_reason = ""

    def _gate(self, tc: TestCase) -> str | None:
        if tc.needs == "factory_reset" and not self.allow_factory_reset:
            return "needs --allow-factory-reset"
        if tc.needs == "flash" and not self.allow_flash:
            return "needs --allow-flash"
        if tc.flash and self.flash_used + tc.flash > self.flash_cap:
            return f"flash budget exhausted ({self.flash_used}+{tc.flash} > {self.flash_cap})"
        return None

    def run(self, cases: list[TestCase]):
        for tc in cases:
            if self.aborted:
                self.results.append(Result(tc.name, tc.group, "SKIP", "run aborted"))
                continue
            gate = self._gate(tc)
            if gate:
                self.results.append(Result(tc.name, tc.group, "SKIP", gate))
                self._emit(self.results[-1])
                continue
            self._run_one(tc)
        return self.results

    def _run_one(self, tc: TestCase):
        chk = Check()
        t0 = time.monotonic()
        reenum_before = self.dev.reenumerations
        status, detail = "PASS", ""
        try:
            tc.fn(self.dev, self.profile, chk)
            if chk.failures:
                status = "FAIL"
                detail = " | ".join(chk.failures[:12])
                if len(chk.failures) > 12:
                    detail += f" | (+{len(chk.failures) - 12} more)"
        except Skip as e:
            status = "SKIP"
            detail = str(e)
        except Disconnected as e:
            status = "ERROR"
            detail = f"DEVICE LOST: {e}"
            self.aborted = True
            self.abort_reason = detail
        except Stall as e:
            status = "ERROR"
            detail = f"unexpected STALL escaped test: {e}"
        except Exception as e:  # noqa: BLE001
            status = "ERROR"
            detail = f"{type(e).__name__}: {e}\n{traceback.format_exc(limit=3)}"
        dur = (time.monotonic() - t0) * 1000.0

        if status == "PASS" and tc.flash:
            self.flash_used += tc.flash

        # Liveness sentinel after any mutating test.  Use the tolerant poll so a
        # transient deferred-reset busy window is not mistaken for a crash; only
        # a device that never recovers within the window aborts the run.
        if tc.mutating and not self.aborted:
            if not self.dev.wait_ready(timeout_s=4.0):
                status = "ERROR"
                detail = (detail + " | " if detail else "") + "DEVICE UNRESPONSIVE after test (liveness poll timed out)"
                self.aborted = True
                self.abort_reason = f"{tc.name}: device unresponsive after mutating test"

        notes = list(chk.notes)
        reenum_delta = self.dev.reenumerations - reenum_before
        if reenum_delta:
            notes.append(f"⚠ {reenum_delta} USB re-enumeration(s) occurred during this test")
        res = Result(tc.name, tc.group, status, detail, notes, dur)
        self.results.append(res)
        self._emit(res)

    def _emit(self, r: Result):
        if not self.verbose:
            return
        icon = {"PASS": "✓", "FAIL": "✗", "SKIP": "–", "ERROR": "!"}[r.status]
        line = f"  {icon} [{r.group}] {r.name}"
        if r.status != "PASS":
            line += f"  — {r.status}: {r.detail.splitlines()[0] if r.detail else ''}"
        print(line, flush=True)

    # -- reporting -----------------------------------------------------------

    def counts(self) -> dict:
        c = {"PASS": 0, "FAIL": 0, "SKIP": 0, "ERROR": 0}
        for r in self.results:
            c[r.status] += 1
        return c

    def to_json(self) -> dict:
        return {
            "device": self.profile.summary(),
            "platform": self.profile.platform_name,
            "fw": self.profile.fw_str,
            "counts": self.counts(),
            "flash_used": self.flash_used,
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "transfers": {"gets": self.dev.gets, "sets": self.dev.sets,
                          "reenumerations": self.dev.reenumerations},
            "results": [
                {"name": r.name, "group": r.group, "status": r.status,
                 "detail": r.detail, "notes": r.notes, "duration_ms": round(r.duration_ms, 1)}
                for r in self.results
            ],
        }

    def to_markdown(self, title="DSPi Control-Plane QA Report", timestamp="") -> str:
        c = self.counts()
        total = sum(c.values())
        lines = [f"# {title}", ""]
        if timestamp:
            lines.append(f"*Generated: {timestamp}*  ")
        lines += [
            f"*Device: {self.profile.summary()}*  ",
            f"*Transfers: {self.dev.gets} GET / {self.dev.sets} SET · flash erases used: {self.flash_used}"
            + (f" · USB re-enumerations: {self.dev.reenumerations}" if self.dev.reenumerations else "") + "*",
            "",
            f"**{c['PASS']}/{total} PASS** · {c['FAIL']} FAIL · {c['ERROR']} ERROR · {c['SKIP']} SKIP",
            "",
        ]
        if self.aborted:
            lines += [f"> ⚠️ **Run aborted:** {self.abort_reason}", ""]

        # Group rollup.
        lines += ["## Summary by group", "", "| Group | Pass | Fail | Error | Skip |", "|---|--:|--:|--:|--:|"]
        groups: dict[str, dict] = {}
        for r in self.results:
            g = groups.setdefault(r.group, {"PASS": 0, "FAIL": 0, "SKIP": 0, "ERROR": 0})
            g[r.status] += 1
        for g, cc in groups.items():
            lines.append(f"| {g} | {cc['PASS']} | {cc['FAIL']} | {cc['ERROR']} | {cc['SKIP']} |")
        lines.append("")

        # Failures/errors first, detailed.
        bad = [r for r in self.results if r.status in ("FAIL", "ERROR")]
        if bad:
            lines += ["## Failures & errors", ""]
            for r in bad:
                lines.append(f"### {r.status}: `{r.name}` ({r.group})")
                lines.append("")
                lines.append("```")
                lines.append(r.detail or "(no detail)")
                lines.append("```")
                lines.append("")

        # Notes & observations (re-enumerations, skipped sub-checks, etc.).
        noted = [r for r in self.results if r.notes]
        if noted:
            lines += ["## Notes & observations", ""]
            for r in noted:
                for n in r.notes:
                    lines.append(f"- `{r.name}` ({r.group}): {n}")
            lines.append("")

        # Full table.
        lines += ["## All tests", "", "| Status | Group | Test | Detail |", "|---|---|---|---|"]
        for r in self.results:
            d = (r.detail or "").splitlines()[0] if r.detail else ""
            d = d.replace("|", "\\|")[:160]
            lines.append(f"| {r.status} | {r.group} | {r.name} | {d} |")
        lines.append("")
        return "\n".join(lines)
