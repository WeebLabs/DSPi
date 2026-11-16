"""
Golden capture of the vendor protocol: capture, compare and selfcheck. See README.md.

Every device request goes through `Golden.raw`, which calls the module-level TRANSPORT. A later
phase swaps in a translating transport with --transport module:function.
"""

from __future__ import annotations

import argparse
import datetime
import importlib
import json
import struct
import sys
import time
from pathlib import Path

from ..device import DspiDevice, Disconnected, Stall, Timeout
from . import script as S

HERE = Path(__file__).resolve().parent
OPS = S.LEGACY
PLATFORM_BY_ID = {v["id"]: k for k, v in S.PLATFORMS.items()}
CS_PENDING = 0x16
BULK_LEN = 6136
LIMITER_OFF = 6028       # WireBulkParams.limiter (V32)
# Physical IO sections of WireBulkParams V32 (pins, i2s_config, input_config, dac_hw_mute,
# adat_config). A baseline push in with-preset mode applies them.
IO_SECTIONS = [(816, 824), (4632, 4648), (4716, 4732), (4764, 4780), (5868, 5876)]


def legacy_transport(dev, direction, code, wvalue, data):
    """Send one legacy 1.1.6 request unchanged. IN returns bytes, OUT returns the byte count."""
    if direction == "IN":
        return bytes(DspiDevice.get(dev, code, data, wvalue))
    return DspiDevice.set(dev, code, data, wvalue)


TRANSPORT = legacy_transport


def load_transport(spec):
    mod, _, attr = spec.partition(":")
    return getattr(importlib.import_module(mod), attr)


class Golden:
    def __init__(self, dev, plan, tiers, log=print):
        self.dev = dev
        self.plan = plan
        self.tiers = tiers
        self.log = log
        self.p = S.PLATFORMS[plan["platform"]]
        self.exempt = set()     # restore diffs that are known and explained
        self.force_io = False
        self.mask = load_mask(platform=plan["platform"])

    # -- transport ----------------------------------------------------------

    def raw(self, direction, op, wvalue=0, data=0):
        return TRANSPORT(self.dev, direction, OPS[op], wvalue, data)

    def get(self, op, length, wvalue=0):
        return bytes(self.raw("IN", op, wvalue, length))

    def put(self, op, payload, wvalue=0):
        return self.raw("OUT", op, wvalue, bytes(payload))

    def run_entry(self, e, payload=None):
        """Issue one script entry; returns reply hex, "ACK", "STALL" or "TIMEOUT"."""
        try:
            if e["dir"] == "W":
                data = payload if payload is not None else bytes.fromhex(e["payload"])
                self.raw("OUT", e["op"], e["wvalue"], data)
                return "ACK"
            return self.raw("IN", e["op"], e["wvalue"], e["len"]).hex()
        except Stall:
            return "STALL"
        except Timeout:
            return "TIMEOUT"

    def wait_ready(self, timeout_s=5.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            try:
                if len(self.get("GET_PLATFORM", 7)) >= 4:
                    return True
            except (Stall, Timeout):
                pass
            time.sleep(0.05)
        return False

    def cs_settle(self, timeout_s=3.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            try:
                st = self.get("GET_CS_STATUS", 41)
                if st[0] != CS_PENDING:
                    return st
            except (Stall, Timeout):
                pass
            time.sleep(0.01)
        return None

    def settle(self, e):
        s = e.get("settle") or {}
        if s.get("delay_ms"):
            time.sleep(s["delay_ms"] / 1000.0)
        if s.get("poll") == "ready":
            self.wait_ready()
        elif s.get("poll") == "cs":
            st = self.cs_settle()
            return "TIMEOUT" if st is None else st[:4].hex()
        return None

    # -- sequence -----------------------------------------------------------

    def run_reads(self):
        return {S.read_key(e): self.run_entry(e) for e in self.plan["reads"]}

    def run_list(self, entries, prefix, baseline_bulk=None):
        out = []
        for i, e in enumerate(entries):
            payload = baseline_bulk if e.get("payload_ref") == "baseline_bulk" else None
            rec = {"id": f"{prefix}:{i:03d}", "op": e["op"], "wvalue": e["wvalue"],
                   "result": self.run_entry(e, payload)}
            cs = self.settle(e)
            if cs is not None:
                rec["cs"] = cs
            out.append(rec)
        return out

    def apply_limiter(self, bulk):
        """Bulk SET skips the limiter in independent output-config mode, so write it explicitly."""
        rows = [struct.unpack_from("<BBxxff", bulk, LIMITER_OFF + 12 * o) for o in range(self.p["nout"])]
        for o in range(self.p["nout"]):
            self.put("LIMITER", struct.pack("<f", 0.0), (o << 8) | 3)
        for o, (en, _link, thr, rel) in enumerate(rows):
            self.put("LIMITER", struct.pack("<f", float(en)), (o << 8) | 0)
            self.put("LIMITER", struct.pack("<f", thr), (o << 8) | 1)
            self.put("LIMITER", struct.pack("<f", rel), (o << 8) | 2)
        for o, (_en, link, _thr, _rel) in enumerate(rows):
            if link:
                self.put("LIMITER", struct.pack("<f", float(link)), (o << 8) | 3)

    def push_bulk(self, bulk):
        self.put("SET_ALL_PARAMS", bulk)
        time.sleep(0.4)
        self.wait_ready()
        if self.get("GET_OUTPUT_CONFIG_MODE", 1)[0] == 0:
            self.apply_limiter(bulk)

    def other_host_reads(self):
        """GET_BUFFER_STATS counts every read; a jump means another host is polling the device."""
        seq = []
        for _ in range(2):
            seq.append(struct.unpack_from("<H", self.get("GET_BUFFER_STATS", 44), 2)[0])
            time.sleep(0.3)
        return (seq[1] - seq[0] - 1) & 0xFFFF

    def precheck(self, st0):
        if "flash" in self.tiers and bytes.fromhex(st0["GET_CS_STATUS@0x0000"])[3]:
            raise SystemExit("Control Surfaces has an unsaved preview; CS_SAVE in the restore would persist it")
        if "reconfig" in self.tiers:
            types = [st0[f"GET_OUTPUT_TYPE@0x{s:04X}"] for s in range(self.p["nspdif"])]
            if st0["GET_INPUT_SOURCE@0x0000"] != "00" or any(t != "00" for t in types):
                raise SystemExit("--allow-reconfig needs USB as the input source and no I2S output slot")
            # Some original pins cannot be re-set through their own command once another
            # function owns the GPIO, so the bulk SET (applied only in with-preset mode) is the backstop.
            if st0["GET_OUTPUT_CONFIG_MODE@0x0000"] != "01":
                raise SystemExit("--allow-reconfig needs output-config mode 1 (with-preset)")

    def sequence(self, baseline_bulk=None):
        """state0, baseline, state A, writes, state B, then restore and verify against state0."""
        res = {}
        self.log("reading initial state")
        st0 = self.run_reads()
        self.precheck(st0)
        bulk0 = bytes.fromhex(st0["GET_ALL_PARAMS@0x0000"])
        if len(bulk0) != BULK_LEN:
            raise SystemExit(f"GET_ALL_PARAMS returned {len(bulk0)} bytes, expected {BULK_LEN}")
        base = baseline_bulk if baseline_bulk is not None else bulk0
        moved = [f"{a}-{b}" for a, b in IO_SECTIONS if base[a:b] != bulk0[a:b]]
        if moved and not self.force_io:
            raise SystemExit(f"the reference baseline would change this device's IO config (bulk bytes "
                             f"{', '.join(moved)}); compare on the capture device or pass --force")
        res["baseline_bulk"] = base.hex()
        try:
            self.log("applying baseline")
            res["baseline"] = self.run_list(self.plan["baseline"], "baseline", base)
            if self.get("GET_OUTPUT_CONFIG_MODE", 1)[0] == 0:
                self.apply_limiter(base)
            self.log("reading state A")
            res["state_a"] = self.run_reads()
            res["writes"] = []
            for tier in ("default", "flash", "reconfig"):
                if tier in self.tiers:
                    self.log(f"running {tier} writes")
                    res["writes"] += self.run_list(self.plan["writes"][tier], tier)
            time.sleep(1.0)
            self.wait_ready()
            self.log("reading state B")
            res["state_b"] = self.run_reads()
        finally:
            self.log("restoring")
            self.restore(st0, bulk0)
            stR = self.run_reads()
            res["restore_diffs"] = [d for d in diff_states(st0, stR, self.mask) if d[0] not in self.exempt]
            res["restore_exempt"] = sorted(self.exempt)
        return res

    # -- restore ------------------------------------------------------------

    def _name_payload(self, hexreply):
        raw = bytes.fromhex(hexreply).split(b"\0", 1)[0]
        return raw if raw else b"\0"

    def restore(self, st0, bulk0):
        if "reconfig" in self.tiers:
            self.restore_reconfig(st0)
        if "flash" in self.tiers:
            self.restore_flash(st0)
        self.push_bulk(bulk0)
        for op, n in [("SIGGEN_CONTROL", 2), ("RTA_CONTROL", 0)]:
            try:
                self.get(op, 1, n)
            except Stall:
                pass
        for op, key in [("SIGGEN_SET_CONFIG", "SIGGEN_GET_CONFIG@0x0000"),
                        ("RTA_SET_CONFIG", "RTA_GET_CONFIG@0x0000"),
                        ("SET_SUBHARM_SOLO", "GET_SUBHARM_SOLO@0x0000")]:
            try:
                self.put(op, bytes.fromhex(st0[key]))
            except Stall:
                if op != "SIGGEN_SET_CONFIG":
                    self.log(f"warning: {op} restore stalled")
                    continue
                # The boot-time siggen config is not settable; leave the canonical one.
                self.put(op, S.SIGGEN_CANON)
                self.exempt.add("SIGGEN_GET_CONFIG@0x0000")
                self.exempt.add("SIGGEN_GET_STATUS@0x0000")
        time.sleep(0.1)
        self.wait_ready()

    def restore_cs(self, st0):
        """Rewrite every CS record the flash tier touched, then CS_SAVE to clear the dirty flag."""
        def cs(op, payload, wvalue=0):
            self.put(op, payload, wvalue)
            st = self.cs_settle()
            if st is None or st[0] != 0:
                self.log(f"warning: restore {op} 0x{wvalue:04X} status {st[:2].hex() if st else 'timeout'}")

        b = lambda key: bytes.fromhex(st0[key])
        cs("SET_CS_BINDING", b("GET_CS_BINDING@0x000F"), 15)
        cs("SET_CS_NAME", self._name_payload(st0["GET_CS_NAME@0x000E"]), 14)
        cs("SET_CS_NAME", self._name_payload(st0["GET_CS_NAME@0x000F"]), 15)
        cs("SET_CS_IR_CMD", b("GET_CS_IR_CMD@0x0003"), 3)
        cs("SET_CS_GROUP", b("GET_CS_GROUP@0x0002"), 2)
        macro = b("GET_CS_MACRO@0x0005")
        for step in (0, 1):
            cs("SET_CS_MACRO_STEP", macro[36 + 12 * step:48 + 12 * step], (step << 8) | 5)
        cs("SET_CS_MACRO", macro[:36], 5)
        cs("SET_CS_DISPLAY_CFG", b("GET_CS_DISPLAY_CFG@0x0000")[4:16])
        cs("SET_CS_DISPLAY_PAGE", b("GET_CS_DISPLAY_PAGE@0x0009"), 9)
        self.get("CS_SAVE", 1)
        time.sleep(0.25)
        self.wait_ready()
        st = self.cs_settle()
        if st is None or st[0] != 0 or st[3] != 0:
            self.log(f"warning: CS_SAVE ended with status {st[:4].hex() if st else 'timeout'}")

    def restore_flash(self, st0):
        def flash(op, payload, wvalue=0):
            self.put(op, payload, wvalue)
            time.sleep(0.25)
            self.wait_ready()

        self.restore_cs(st0)

        flash("SET_DEVICE_NAME", self._name_payload(st0["GET_DEVICE_NAME@0x0000"]))
        for s in (0, 3, 6):
            flash("SET_INPUT_SOURCE_NAME", self._name_payload(st0[f"GET_INPUT_SOURCE_NAME@0x{s:04X}"]), s)
        flash("PRESET_SET_NAME", self._name_payload(st0["PRESET_GET_NAME@0x0009"]), 9)
        flash("PRESET_SET_STARTUP", bytes.fromhex(st0["PRESET_GET_STARTUP@0x0000"])[:2])
        flash("SET_OUTPUT_CONFIG_MODE", bytes.fromhex(st0["GET_OUTPUT_CONFIG_MODE@0x0000"]))
        flash("SET_MASTER_VOLUME_MODE", bytes.fromhex(st0["GET_MASTER_VOLUME_MODE@0x0000"]))
        self.put("SET_MASTER_VOLUME", bytes.fromhex(st0["GET_SAVED_MASTER_VOLUME@0x0000"]))
        self.get("SAVE_MASTER_VOLUME", 1)
        time.sleep(0.25)
        self.wait_ready()

    def restore_reconfig(self, st0):
        def b(key):
            return bytes.fromhex(st0[key])

        def act(op, wvalue):
            try:
                st = self.get(op, 1, wvalue)[0]
                if st != 0:
                    self.log(f"note: restore {op} 0x{wvalue:04X} refused (status {st}); "
                             f"the bulk restore puts it back")
            except Stall:
                self.log(f"warning: restore {op} 0x{wvalue:04X} stalled")

        act("SET_ADAT_PIN", b("GET_ADAT_PIN@0x0000")[0])
        act("SET_MCK_PIN", b("GET_MCK_PIN@0x0000")[0])
        act("SET_MCK_MULTIPLIER", b("GET_MCK_MULTIPLIER@0x0000")[0])
        act("SET_I2S_CLOCK_PIN_MODE", b("GET_I2S_CLOCK_PIN_MODE@0x0000")[0])
        act("SET_I2S_BCK_PIN", b("GET_I2S_BCK_PIN@0x0000")[0])
        act("SET_I2S_BCK_PIN", (1 << 8) | b("GET_I2S_BCK_PIN@0x0001")[0])
        act("SET_I2S_INPUT_CHANNELS", b("GET_I2S_INPUT_CHANNELS@0x0000")[0])
        for pair in range(4):
            act("SET_I2S_RX_PIN", (pair << 8) | b(f"GET_I2S_RX_PIN@0x{pair:04X}")[0])
        cfg = b("GET_SPDIF_INPUT_CONFIG@0x0000")
        for idx in range(4):
            act("SET_SPDIF_RX_PIN", (idx << 8) | cfg[2 + idx])
        for idx in range(1, 4):
            act("SET_SPDIF_INPUT_ENABLE", (idx << 8) | ((cfg[1] >> idx) & 1))
        act("SET_ADAT_INPUT_ENABLE", b("GET_ADAT_INPUT_ENABLE@0x0000")[0])
        act("SET_ADAT_INPUT_PIN", b("GET_ADAT_INPUT_PIN@0x0000")[0])
        act("SET_ADAT_INPUT_CLOCK_MODE", b("GET_ADAT_INPUT_CLOCK_MODE@0x0000")[0])
        self.put("SET_I2S_CLOCK_MODE", b("GET_I2S_CLOCK_MODE@0x0000"))
        self.put("SET_INPUT_RATE", b("GET_INPUT_RATE@0x0000")[4:8])
        time.sleep(0.2)
        self.wait_ready()


# ---------------------------------------------------------------- mask and diff

def load_mask(path=HERE / "mask.json", platform=None):
    if not Path(path).is_file():
        return []
    entries = json.loads(Path(path).read_text())["entries"]
    return [m for m in entries if m.get("platform") in (None, platform)]


def masked_offsets(mask, op, wvalue, length):
    hit = set()
    exact = [m for m in mask if m["op"] == op and m["wvalue"] == wvalue]
    for m in exact or [m for m in mask if m["op"] == op and m["wvalue"] == "*"]:
        stride = m.get("stride") or length or 1
        for base in range(0, length, stride):
            for start, end in m["ranges"]:
                stop = stride if end is None else end
                hit.update(range(base + start, min(base + stop, length)))
    return hit


def _ranges(offsets):
    out, run = [], None
    for o in sorted(offsets):
        if run and o == run[1]:
            run[1] = o + 1
        else:
            run = [o, o + 1]
            out.append(run)
    return out


def diff_result(op, wvalue, ref, got, mask):
    """Unmasked differences between two results: a list of (offset or None, expected, got)."""
    if ref == got:
        return []
    if ref in ("STALL", "TIMEOUT", "ACK") or got in ("STALL", "TIMEOUT", "ACK"):
        return [(None, ref, got)]
    a, b = bytes.fromhex(ref), bytes.fromhex(got)
    if len(a) != len(b):
        return [(None, f"{len(a)} bytes", f"{len(b)} bytes")]
    skip = masked_offsets(mask, op, wvalue, len(a))
    return [(i, a[i], b[i]) for i in range(len(a)) if a[i] != b[i] and i not in skip]


def _split_key(key):
    op, _, wv = key.partition("@")
    return op, int(wv, 16)


def diff_states(ref, got, mask):
    out = []
    for key, r in ref.items():
        op, wv = _split_key(key)
        d = diff_result(op, wv, r, got.get(key, "MISSING"), mask)
        if d:
            out.append((key, d))
    return out


def diff_lists(ref, got, mask):
    out = []
    for r, g in zip(ref, got):
        d = diff_result(r["op"], r["wvalue"], r["result"], g["result"], mask)
        if r.get("cs") != g.get("cs"):
            d = d + [("cs", r.get("cs"), g.get("cs"))]
        if d:
            out.append((r["id"] + " " + r["op"] + f"@0x{r['wvalue']:04X}", d))
    if len(ref) != len(got):
        out.append(("length", [(None, f"{len(ref)} entries", f"{len(got)} entries")]))
    return out


def compare_runs(ref, got, mask):
    sections = []
    sections.append(("baseline", diff_lists(ref["baseline"], got["baseline"], mask)))
    sections.append(("state A", diff_states(ref["state_a"], got["state_a"], mask)))
    sections.append(("writes", diff_lists(ref["writes"], got["writes"], mask)))
    sections.append(("state B", diff_states(ref["state_b"], got["state_b"], mask)))
    return sections


def print_diffs(sections, limit=12):
    total = 0
    for name, diffs in sections:
        for label, d in diffs:
            total += 1
            byte_diffs = [x for x in d if isinstance(x[0], int)]
            other = [x for x in d if not isinstance(x[0], int)]
            for off, exp, got in other:
                print(f"  {name:8s} {label}: {off or 'reply'} expected {exp} got {got}")
            if byte_diffs:
                rngs = _ranges(o for o, _, _ in byte_diffs)
                shown = ", ".join(f"{o}: {e:02x}->{g:02x}" for o, e, g in byte_diffs[:limit])
                more = "" if len(byte_diffs) <= limit else f" (+{len(byte_diffs) - limit} more)"
                print(f"  {name:8s} {label}: offsets {rngs} {shown}{more}")
    return total


# ---------------------------------------------------------------- file I/O

def write_capture(path, doc):
    """One request per line so a diff of two captures stays readable."""
    lines = ["{"]
    keys = list(doc)
    for i, k in enumerate(keys):
        comma = "," if i < len(keys) - 1 else ""
        v = doc[k]
        if isinstance(v, dict) and k.startswith("state"):
            lines.append(f" {json.dumps(k)}: {{")
            items = list(v.items())
            for j, (kk, vv) in enumerate(items):
                lines.append(f"  {json.dumps(kk)}: {json.dumps(vv)}" + ("," if j < len(items) - 1 else ""))
            lines.append(" }" + comma)
        elif isinstance(v, list) and v and isinstance(v[0], dict):
            lines.append(f" {json.dumps(k)}: [")
            for j, vv in enumerate(v):
                lines.append("  " + json.dumps(vv) + ("," if j < len(v) - 1 else ""))
            lines.append(" ]" + comma)
        else:
            lines.append(f" {json.dumps(k)}: {json.dumps(v)}{comma}")
    lines.append("}")
    Path(path).write_text("\n".join(lines) + "\n")


def identify(g):
    plat = g.get("GET_PLATFORM", 7)
    try:
        bi = g.get("GET_BUILD_INFO", 64)[:48].split(b"\0")[0].decode("ascii", "replace")
    except Stall:
        bi = ""
    fw = f"{plat[1]}.{plat[4]}.{plat[5]}" + (f"-beta{plat[6]}" if len(plat) > 6 and plat[6] else "")
    return plat, fw, bi


def open_device(args, tiers):
    dev = DspiDevice()
    probe = Golden(dev, {"platform": "RP2350", "reads": []}, tiers)
    plat, fw, bi = identify(probe)
    name = PLATFORM_BY_ID.get(plat[0])
    if name is None:
        raise SystemExit(f"unknown platform id {plat[0]}")
    plan, sha = S.load(name)
    print(f"device: {name} fw {fw} build {bi or '?'}")
    g = Golden(dev, plan, tiers)
    g.mask = load_mask(args.mask, name)
    others = g.other_host_reads()
    if others:
        msg = f"another host is polling the device ({others} reads in 0.3 s); close DSPi Console"
        if not args.allow_other_hosts:
            raise SystemExit(msg + " or pass --allow-other-hosts")
        print("warning: " + msg)
    return g, plat, fw, bi, sha


def tiers_from_args(args):
    t = ["default"]
    if args.allow_flash:
        t.append("flash")
    if args.allow_reconfig:
        t.append("reconfig")
    return t


def report_restore(res):
    for key in res.get("restore_exempt", []):
        print(f"restore: {key} left at the canonical value (boot default is not settable)")
    diffs = res.get("restore_diffs", [])
    if not diffs:
        print("restore: device state matches the initial read (masked fields excepted)")
        return True
    print(f"restore: {len(diffs)} request(s) differ from the initial read")
    print_diffs([("restore", diffs)])
    return False


def cmd_capture(args):
    tiers = tiers_from_args(args)
    g, plat, fw, bi, sha = open_device(args, tiers)
    if args.expect_build and not bi.startswith(args.expect_build):
        raise SystemExit(f"build info {bi!r} does not start with {args.expect_build!r}")
    if args.selfcheck:
        return cmd_selfcheck(g)
    res = g.sequence()
    doc = {"format": 1, "captured": datetime.date.today().isoformat(),
           "platform": g.plan["platform"], "platform_reply": plat.hex(), "fw": fw, "build_info": bi,
           "script": S.script_path(g.plan["platform"]).name, "script_sha256": sha, "tiers": tiers}
    doc.update({k: res[k] for k in ("baseline_bulk", "baseline", "writes", "state_a", "state_b")})
    write_capture(args.out, doc)
    print(f"wrote {args.out}: {len(res['state_a'])} reads x2, {len(res['writes'])} writes")
    return 0 if report_restore(res) else 3


def cmd_compare(args):
    ref = json.loads(Path(args.ref).read_text())
    need = set(ref["tiers"])
    tiers = tiers_from_args(args)
    if need != set(tiers):
        raise SystemExit(f"reference was captured with tiers {ref['tiers']}; pass the matching "
                         f"--allow-flash / --allow-reconfig flags")
    g, plat, fw, bi, sha = open_device(args, tiers)
    if ref["platform"] != g.plan["platform"]:
        raise SystemExit(f"reference is {ref['platform']}, device is {g.plan['platform']}")
    if ref["script_sha256"] != sha and not args.force:
        raise SystemExit("script file changed since the reference was captured (use --force)")
    g.force_io = args.force
    res = g.sequence(bytes.fromhex(ref["baseline_bulk"]))
    n = print_diffs(compare_runs(ref, res, g.mask))
    ok_restore = report_restore(res)
    print(f"compare: {n} unexplained difference(s) against {args.ref} ({ref['fw']} {ref['build_info']})")
    return 1 if n else (0 if ok_restore else 3)


def cmd_selfcheck(g):
    runs = []
    for i in range(2):
        print(f"selfcheck run {i + 1}")
        runs.append(g.sequence())
    sections = compare_runs(runs[0], runs[1], [])
    n = print_diffs(sections, limit=6)
    suggest = {}
    for _name, diffs in sections:
        for label, d in diffs:
            key = label.split()[-1] if " " in label else label
            offs = [o for o, _, _ in d if isinstance(o, int)]
            if offs:
                suggest.setdefault(key, set()).update(offs)
    for key, offs in sorted(suggest.items()):
        print(f"  volatile {key}: {_ranges(offs)}")
    print(f"selfcheck: {n} request(s) differ between two back-to-back runs")
    ok = all(report_restore(r) for r in runs)
    return 0 if ok else 3


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.dspi_test.golden.capture")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("capture", "compare"):
        sp = sub.add_parser(name)
        sp.add_argument("--allow-flash", action="store_true", help="include the flash-writing tier")
        sp.add_argument("--allow-reconfig", action="store_true", help="include the IO reconfig tier")
        sp.add_argument("--transport", help="module:function replacing the legacy transport")
        sp.add_argument("--mask", default=str(HERE / "mask.json"))
        sp.add_argument("--allow-other-hosts", action="store_true",
                        help="run even if another host is polling (chunked bulk reads may then differ)")
    cap = sub.choices["capture"]
    cap.add_argument("--out", help="capture file to write")
    cap.add_argument("--selfcheck", action="store_true", help="run twice and list volatile bytes")
    cap.add_argument("--expect-build", help="refuse unless GET_BUILD_INFO starts with this")
    cmp_ = sub.choices["compare"]
    cmp_.add_argument("--ref", required=True)
    cmp_.add_argument("--force", action="store_true",
                      help="compare even if the script hash or the device IO config differs")
    args = ap.parse_args(argv)
    global TRANSPORT
    if args.transport:
        TRANSPORT = load_transport(args.transport)
    if args.cmd == "capture" and not args.out and not args.selfcheck:
        ap.error("capture needs --out or --selfcheck")
    try:
        return cmd_capture(args) if args.cmd == "capture" else cmd_compare(args)
    except Disconnected as e:
        print(f"device disconnected: {e}")
        return 4


if __name__ == "__main__":
    sys.exit(main())
