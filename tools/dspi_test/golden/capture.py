"""
Golden regression of the paged vendor protocol: capture, compare and selfcheck. See README.md.
Every request is a native paged GET or SET from script_<platform>.json.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import struct
import subprocess
import sys
import time
from pathlib import Path

from ..device import DspiDevice, Disconnected, PagedStall, ProtocolError, Stall, Timeout

HERE = Path(__file__).resolve().parent
PLATFORMS = {
    "RP2350": dict(id=1, nout=9, nspdif=4),
    "RP2040": dict(id=0, nout=5, nspdif=2),
}
PLATFORM_BY_ID = {v["id"]: k for k, v in PLATFORMS.items()}
FORMAT = 2
CS_PENDING = 0x16
CS_STATUS_LEN = 41
BULK_LEN = 6136
BULK_BUSY_RETRIES = 10
LIMITER_OFF = 6028       # WireBulkParams.limiter (V32)
# Physical IO sections of WireBulkParams V32 (pins, i2s_config, input_config, dac_hw_mute,
# adat_config). A baseline push in with-preset mode applies them.
IO_SECTIONS = [(816, 824), (4632, 4648), (4716, 4732), (4764, 4780), (5868, 5876)]
# 1 kHz sine at -20 dBFS on outputs 0-1, staged only: MEAS.SIGGEN_CONFIG refuses the all-zero boot config.
SIGGEN_CANON = struct.pack("<BBHHBBfIHHffff", 1, 0, 0x0003, 0, 0, 0, -20.0, 0, 0, 0, 1000.0, 0, 0, 0)


def key(cmd, addr):
    """Identity of a read; the mask and the reference are keyed by it."""
    return f"{cmd}@0x{addr:04X}"


def is_bytes(result):
    """Replies are lowercase hex; OK, VC_*, STALL, TIMEOUT and MISSING are uppercase tokens."""
    return all(c in "0123456789abcdef" for c in result)


def script_path(platform):
    return HERE / f"script_{platform.lower()}.json"


def load_script(platform):
    raw = script_path(platform).read_bytes()
    plan = json.loads(raw)
    if plan.get("format") != FORMAT:
        raise SystemExit(f"{script_path(platform).name} is not a format {FORMAT} (paged) script")
    return plan, hashlib.sha256(raw).hexdigest()


class Golden:
    def __init__(self, dev, plan, tiers, mask, log=print):
        self.dev = dev
        self.plan = plan
        self.tiers = tiers
        self.mask = mask
        self.log = log
        self.p = PLATFORMS[plan["platform"]]
        self.exempt = set()     # restore diffs that are known and explained
        self.force_io = False

    # -- requests -----------------------------------------------------------

    def get(self, cmd, length, addr=0):
        return self.dev.pget(cmd, addr, length)

    def put(self, cmd, payload=b"", addr=0):
        return self.dev.pset(cmd, bytes(payload), addr)

    def run_entry(self, e, payload=None):
        """Issue one script entry; returns reply hex, OK, the VC_* result, STALL or TIMEOUT."""
        try:
            if e["dir"] == "S":
                data = payload if payload is not None else bytes.fromhex(e["payload"])
                self.put(e["cmd"], data, e["addr"])
                return "OK"
            return self.get(e["cmd"], e["len"], e["addr"]).hex()
        except PagedStall as s:
            return s.result_name or "STALL"     # no matching LAST_RESULT: another host interfered
        except Timeout:
            return "TIMEOUT"

    def wait_ready(self, timeout_s=5.0):
        return self.dev.wait_ready(timeout_s, 0.05)

    def cs_settle(self, timeout_s=3.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            try:
                st = self.get("CS.STATUS", CS_STATUS_LEN)
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
        return {key(e["cmd"], e["addr"]): self.run_entry(e) for e in self.plan["reads"]}

    def run_list(self, entries, prefix, baseline_bulk=None):
        out = []
        for i, e in enumerate(entries):
            payload = baseline_bulk if e.get("payload_ref") == "baseline_bulk" else None
            rec = {"id": f"{prefix}:{i:03d}", "cmd": e["cmd"], "addr": e["addr"],
                   "result": self.run_entry(e, payload)}
            cs = self.settle(e)
            if cs is not None:
                rec["cs"] = cs
            out.append(rec)
        return out

    def apply_limiter(self, bulk):
        """The bulk SET skips the limiter in independent output-config mode, so write it explicitly."""
        rows = [struct.unpack_from("<BBxxff", bulk, LIMITER_OFF + 12 * o) for o in range(self.p["nout"])]
        f32 = lambda v: struct.pack("<f", float(v))  # noqa: E731
        for o in range(self.p["nout"]):
            self.put("DYN.LIMITER_LINK_GROUP", f32(0), o)
        for o, (en, _link, thr, rel) in enumerate(rows):
            self.put("DYN.LIMITER_ENABLE", f32(en), o)
            self.put("DYN.LIMITER_THRESHOLD", f32(thr), o)
            self.put("DYN.LIMITER_RELEASE", f32(rel), o)
        for o, (_en, link, _thr, _rel) in enumerate(rows):
            if link:
                self.put("DYN.LIMITER_LINK_GROUP", f32(link), o)

    def push_bulk(self, bulk):
        for attempt in range(BULK_BUSY_RETRIES):
            try:
                self.put("STORE.ALL_PARAMS", bulk)
                break
            except PagedStall as s:
                if s.result_name != "VC_ERR_BUSY" or attempt == BULK_BUSY_RETRIES - 1:
                    raise
                time.sleep(0.1)
        time.sleep(0.4)
        self.wait_ready()
        if self.get("STORE.OUTPUT_CONFIG_MODE", 1)[0] == 0:
            self.apply_limiter(bulk)

    def other_host_reads(self):
        """SYS.BUFFER_STATS counts every read; a jump means another host is polling the device."""
        seq = []
        for _ in range(2):
            seq.append(struct.unpack_from("<H", self.get("SYS.BUFFER_STATS", 44), 2)[0])
            time.sleep(0.3)
        return (seq[1] - seq[0] - 1) & 0xFFFF

    def precheck(self, st0):
        b = lambda k: field(st0, k)  # noqa: E731
        if "flash" in self.tiers and b("CS.STATUS@0x0000")[3]:
            raise SystemExit("Control Surfaces has an unsaved preview; CS.SAVE in the restore would persist it")
        if "reconfig" in self.tiers:
            types = [b(key("OUTPUT.TYPE", s)) for s in range(self.p["nspdif"])]
            if b("INPUT.SOURCE@0x0000")[0] != 0 or any(t[0] != 0 for t in types):
                raise SystemExit("--allow-reconfig needs USB as the input source and no I2S output slot")
            # An enabled I2S input block holds a clock pair, so the BCK writes would move held pins.
            if b("INPUT.I2S_RX_ENABLE@0x0000")[0] != 0:
                raise SystemExit("--allow-reconfig needs the I2S input block off")
            # Some original pins cannot be re-set through their own command once another
            # function owns the GPIO, so the bulk SET (applied only in with-preset mode) is the backstop.
            if b("STORE.OUTPUT_CONFIG_MODE@0x0000")[0] != 1:
                raise SystemExit("--allow-reconfig needs output-config mode 1 (with-preset)")

    def sequence(self, baseline_bulk=None):
        """Initial read, baseline, state A, writes, state B, then restore and verify against the initial read."""
        res = {}
        self.log("reading initial state")
        st0 = self.run_reads()
        self.precheck(st0)
        bulk0 = field(st0, "STORE.ALL_PARAMS@0x0000")
        if len(bulk0) != BULK_LEN:
            raise SystemExit(f"STORE.ALL_PARAMS returned {len(bulk0)} bytes, expected {BULK_LEN}")
        base = baseline_bulk if baseline_bulk is not None else bulk0
        moved = [f"{a}-{b}" for a, b in IO_SECTIONS if base[a:b] != bulk0[a:b]]
        if moved and not self.force_io:
            raise SystemExit(f"the reference baseline would change this device's IO config (bulk bytes "
                             f"{', '.join(moved)}); compare on the capture device or pass --force")
        res["baseline_bulk"] = base.hex()
        try:
            self.log("applying baseline")
            res["baseline"] = self.run_list(self.plan["baseline"], "baseline", base)
            if self.get("STORE.OUTPUT_CONFIG_MODE", 1)[0] == 0:
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
            res["restore_diffs"] = [d for d in diff_states(st0, stR, self.mask, "restore")
                                    if d[0] not in self.exempt]
            res["restore_exempt"] = sorted(self.exempt)
        return res

    # -- restore ------------------------------------------------------------

    def restore(self, st0, bulk0):
        if "reconfig" in self.tiers:
            self.restore_reconfig(st0)
        if "flash" in self.tiers:
            self.restore_flash(st0)
        self.push_bulk(bulk0)
        for cmd in ("MEAS.SIGGEN_STOP_NOW", "MEAS.RTA_STOP"):
            try:
                self.put(cmd)
            except PagedStall:
                pass
        for cmd in ("MEAS.SIGGEN_CONFIG", "MEAS.RTA_CONFIG", "FX.SUBHARM_SOLO"):
            try:
                self.put(cmd, field(st0, key(cmd, 0)))
            except PagedStall as s:
                if cmd != "MEAS.SIGGEN_CONFIG":
                    self.log(f"warning: {cmd} restore refused ({s.result_name})")
                    continue
                self.put(cmd, SIGGEN_CANON)
                self.exempt.update({key("MEAS.SIGGEN_CONFIG", 0), key("MEAS.SIGGEN_STATUS", 0)})
        time.sleep(0.1)
        self.wait_ready()

    def restore_cs(self, st0):
        """Rewrite every CS record the flash tier touched, then CS.SAVE to clear the dirty flag.
        CS.REVERT is not used: it leaves stored non-button bindings down with PIN_IN_USE."""
        def cs(cmd, payload, addr=0):
            self.put(cmd, payload, addr)
            st = self.cs_settle()
            if st is None or st[0] != 0:
                self.log(f"warning: restore {cmd} 0x{addr:04X} status {st[:2].hex() if st else 'timeout'}")

        b = lambda k: field(st0, k)  # noqa: E731
        cs("CS.BINDING", b("CS.BINDING@0x000F"), 15)
        cs("CS.SLOT_NAME", name_payload(b("CS.SLOT_NAME@0x000E")), 14)
        cs("CS.SLOT_NAME", name_payload(b("CS.SLOT_NAME@0x000F")), 15)
        cs("CS.IR_CMD", b("CS.IR_CMD@0x0003"), 3)
        cs("CS.GROUP", b("CS.GROUP@0x0002"), 2)
        macro = b("CS.MACRO@0x0005")
        for step in (0, 1):
            cs("CS.MACRO_STEP", macro[36 + 12 * step:48 + 12 * step], (step << 8) | 5)
        cs("CS.MACRO_HEADER", macro[:36], 5)
        cs("CS.DISPLAY_CONFIG", b("CS.DISPLAY_CONFIG@0x0000"))
        cs("CS.DISPLAY_PAGE", b("CS.DISPLAY_PAGE@0x0009"), 9)
        self.put("CS.SAVE")
        time.sleep(0.25)
        self.wait_ready()
        st = self.cs_settle()
        if st is None or st[0] != 0 or st[3] != 0:
            self.log(f"warning: CS.SAVE ended with status {st[:4].hex() if st else 'timeout'}")

    def restore_flash(self, st0):
        def flash(cmd, payload, addr=0):
            self.put(cmd, payload, addr)
            time.sleep(0.25)
            self.wait_ready()

        b = lambda k: field(st0, k)  # noqa: E731
        self.restore_cs(st0)
        flash("SYS.DEVICE_NAME", name_payload(b("SYS.DEVICE_NAME@0x0000")))
        for s in (0, 3, 6):
            flash("INPUT.SOURCE_NAME", name_payload(b(key("INPUT.SOURCE_NAME", s))), s)
        flash("STORE.PRESET_NAME", name_payload(b("STORE.PRESET_NAME@0x0009")), 9)
        flash("STORE.STARTUP", b("STORE.STARTUP@0x0000")[:2])
        flash("STORE.OUTPUT_CONFIG_MODE", b("STORE.OUTPUT_CONFIG_MODE@0x0000"))
        flash("ROUTE.MASTER_VOLUME_MODE", b("ROUTE.MASTER_VOLUME_MODE@0x0000"))
        self.put("ROUTE.MASTER_VOLUME", b("ROUTE.MASTER_VOLUME_SAVED@0x0000"))
        flash("ROUTE.MASTER_VOLUME_SAVE", b"")

    def restore_reconfig(self, st0):
        b = lambda k: field(st0, k)  # noqa: E731

        def put(cmd, payload, addr=0):
            try:
                self.put(cmd, payload, addr)
            except PagedStall as s:
                self.log(f"note: restore {cmd} 0x{addr:04X} refused ({s.result_name}); "
                         f"the bulk restore puts it back")

        def u8(k):
            return b(k)[:1]

        put("OUTPUT.ADAT_TX_PIN", u8("OUTPUT.ADAT_TX_PIN@0x0000"))
        put("CLOCK.MCK_PIN", u8("CLOCK.MCK_PIN@0x0000"))
        put("CLOCK.MCK_MULTIPLIER", u8("CLOCK.MCK_MULTIPLIER@0x0000"))
        put("CLOCK.I2S_CLOCK_PIN_MODE", u8("CLOCK.I2S_CLOCK_PIN_MODE@0x0000"))
        for role in (0, 1):
            put("CLOCK.I2S_BCK_PIN", u8(key("CLOCK.I2S_BCK_PIN", role)), role)
        put("INPUT.I2S_RX_CHANNELS", u8("INPUT.I2S_RX_CHANNELS@0x0000"))
        for pair in range(4):
            put("INPUT.I2S_RX_PIN", u8(key("INPUT.I2S_RX_PIN", pair)), pair)
        # Pins first while the script leaves the S/PDIF block off, then the count before the switch.
        cfg = b("INPUT.SPDIF_RX_CONFIG@0x0000")
        for idx in range(4):
            put("INPUT.SPDIF_RX_PIN", cfg[3 + idx:4 + idx], idx)
        put("INPUT.SPDIF_RX_COUNT", cfg[2:3])
        put("INPUT.SPDIF_RX_ENABLE", cfg[1:2])
        put("INPUT.ADAT_RX_ENABLE", u8("INPUT.ADAT_RX_ENABLE@0x0000"))
        put("INPUT.ADAT_RX_PIN", u8("INPUT.ADAT_RX_PIN@0x0000"))
        put("INPUT.ADAT_RX_CLOCK_MODE", u8("INPUT.ADAT_RX_CLOCK_MODE@0x0000"))
        put("CLOCK.I2S_CLOCK_MODE", u8("CLOCK.I2S_CLOCK_MODE@0x0000"))
        put("INPUT.I2S_RX_RATE", b("INPUT.I2S_RX_RATE@0x0000")[:4])
        time.sleep(0.2)
        self.wait_ready()


def field(state, k):
    """Bytes of one read in a state; a refused read here means the script and firmware disagree."""
    r = state.get(k, "MISSING")
    if not is_bytes(r):
        raise SystemExit(f"{k} answered {r}; the tool needs it to check or restore the device")
    return bytes.fromhex(r)


def name_payload(raw):
    raw = raw.split(b"\0", 1)[0]
    return raw if raw else b"\0"


# ---------------------------------------------------------------- mask and diff

def load_mask(path=HERE / "mask.json", platform=None):
    if not Path(path).is_file():
        return []
    entries = json.loads(Path(path).read_text())["entries"]
    return [m for m in entries if m.get("platform") in (None, platform)]


def masked_offsets(mask, cmd, addr, length, section):
    hit = set()
    mine = [m for m in mask if m["cmd"] == cmd and section in m.get("sections", [section])]
    exact = [m for m in mine if m["addr"] == addr]
    for m in exact or [m for m in mine if m["addr"] == "*"]:
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


def diff_result(cmd, addr, ref, got, mask, section):
    """Unmasked differences between two results: a list of (offset or None, expected, got)."""
    if ref == got:
        return []
    if not (is_bytes(ref) and is_bytes(got)):
        return [(None, ref, got)]
    a, b = bytes.fromhex(ref), bytes.fromhex(got)
    if len(a) != len(b):
        return [(None, f"{len(a)} bytes", f"{len(b)} bytes")]
    skip = masked_offsets(mask, cmd, addr, len(a), section)
    return [(i, a[i], b[i]) for i in range(len(a)) if a[i] != b[i] and i not in skip]


def _split_key(k):
    cmd, _, addr = k.partition("@")
    return cmd, int(addr, 16)


def diff_states(ref, got, mask, section):
    out = []
    for k, r in ref.items():
        cmd, addr = _split_key(k)
        d = diff_result(cmd, addr, r, got.get(k, "MISSING"), mask, section)
        if d:
            out.append((k, d))
    for k in got.keys() - ref.keys():
        out.append((k, [(None, "MISSING", got[k])]))
    return out


def diff_lists(ref, got, mask, section):
    out = []
    for r, g in zip(ref, got):
        d = diff_result(r["cmd"], r["addr"], r["result"], g["result"], mask, section)
        if (r["cmd"], r["addr"]) != (g["cmd"], g["addr"]):
            d = [(None, key(r["cmd"], r["addr"]), key(g["cmd"], g["addr"]))]
        if r.get("cs") != g.get("cs"):
            d = d + [("cs", r.get("cs"), g.get("cs"))]
        if d:
            out.append((r["id"] + " " + key(r["cmd"], r["addr"]), d))
    if len(ref) != len(got):
        out.append(("length", [(None, f"{len(ref)} entries", f"{len(got)} entries")]))
    return out


def compare_runs(ref, got, mask):
    return [("baseline", diff_lists(ref["baseline"], got["baseline"], mask, "baseline")),
            ("state_a", diff_states(ref["state_a"], got["state_a"], mask, "state_a")),
            ("writes", diff_lists(ref["writes"], got["writes"], mask, "writes")),
            ("state_b", diff_states(ref["state_b"], got["state_b"], mask, "state_b"))]


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


def git_describe():
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty", "--abbrev=7"], cwd=HERE,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def open_device(args, tiers):
    try:
        dev = DspiDevice()
        info = dev.protocol_info()
    except ProtocolError as e:
        raise SystemExit(f"refusing to run: {e}") from None
    if info is None:
        raise SystemExit("refusing to run: SYS.PROTOCOL_INFO STALLed, so this firmware (1.1.6 or older) "
                         "has no paged protocol")
    plat = dev.pget("SYS.PLATFORM", 0, 7)
    try:
        bi = dev.pget("SYS.BUILD_INFO", 0, 64)[:48].split(b"\0")[0].decode("ascii", "replace")
    except Stall:
        bi = ""
    fw = f"{plat[1]}.{plat[4]}.{plat[5]}" + (f"-beta{plat[6]}" if len(plat) > 6 and plat[6] else "")
    name = PLATFORM_BY_ID.get(plat[0])
    if name is None:
        raise SystemExit(f"unknown platform id {plat[0]}")
    if args.expect_build and not bi.startswith(args.expect_build):
        raise SystemExit(f"build info {bi!r} does not start with {args.expect_build!r}")
    plan, sha = load_script(name)
    if "reconfig" in tiers and not plan["writes"].get("reconfig"):
        raise SystemExit(f"{name} has no reconfig tier; drop --allow-reconfig")
    print(f"device: {name} fw {fw} build {bi or '?'} protocol {info['major']}.{info['minor']}")
    g = Golden(dev, plan, tiers, load_mask(args.mask, name))
    others = g.other_host_reads()
    if others:
        msg = f"another host is polling the device ({others} reads in 0.3 s); close DSPi Console"
        if not args.allow_other_hosts:
            raise SystemExit(msg + " or pass --allow-other-hosts")
        print("warning: " + msg)
    ident = {"platform": name, "platform_reply": plat.hex(), "fw": fw, "build_info": bi,
             "protocol": f"{info['major']}.{info['minor']}"}
    return g, ident, sha


def tiers_from_args(args):
    t = ["default"]
    if args.allow_flash:
        t.append("flash")
    if args.allow_reconfig:
        t.append("reconfig")
    return t


def report_restore(res):
    for k in res.get("restore_exempt", []):
        print(f"restore: {k} left at the canonical value (boot default is not settable)")
    diffs = res.get("restore_diffs", [])
    if not diffs:
        print("restore: device state matches the initial read (masked fields excepted)")
        return True
    print(f"restore: {len(diffs)} request(s) differ from the initial read")
    print_diffs([("restore", diffs)])
    return False


def cmd_capture(args):
    tiers = tiers_from_args(args)
    g, ident, sha = open_device(args, tiers)
    if args.selfcheck:
        return cmd_selfcheck(g)
    res = g.sequence()
    doc = {"format": FORMAT, "captured": datetime.date.today().isoformat(), **ident,
           "git": git_describe(), "script": script_path(ident["platform"]).name, "script_sha256": sha,
           "tiers": tiers}
    if args.provenance:
        doc["provenance"] = json.loads(Path(args.provenance).read_text())
    doc.update({k: res[k] for k in ("baseline_bulk", "baseline", "writes", "state_a", "state_b")})
    write_capture(args.out, doc)
    print(f"wrote {args.out}: {len(res['state_a'])} reads x2, {len(res['writes'])} writes")
    return 0 if report_restore(res) else 3


def cmd_compare(args):
    ref = json.loads(Path(args.ref).read_text())
    if ref.get("format") != FORMAT:
        raise SystemExit(f"{args.ref} is not a format {FORMAT} (paged) reference")
    tiers = tiers_from_args(args)
    if set(ref["tiers"]) != set(tiers):
        raise SystemExit(f"reference was captured with tiers {ref['tiers']}; pass the matching "
                         f"--allow-flash / --allow-reconfig flags")
    g, ident, sha = open_device(args, tiers)
    if ref["platform"] != ident["platform"]:
        raise SystemExit(f"reference is {ref['platform']}, device is {ident['platform']}")
    if ref["script_sha256"] != sha and not args.force:
        raise SystemExit("script file changed since the reference was captured (use --force)")
    g.force_io = args.force
    res = g.sequence(bytes.fromhex(ref["baseline_bulk"]))
    n = print_diffs(compare_runs(ref, res, g.mask))
    ok_restore = report_restore(res)
    print(f"compare: {n} difference(s) against {args.ref} ({ref['fw']} {ref['build_info']})")
    return 1 if n else (0 if ok_restore else 3)


def cmd_selfcheck(g):
    runs = []
    for i in range(2):
        print(f"selfcheck run {i + 1}")
        runs.append(g.sequence())
    sections = compare_runs(runs[0], runs[1], [])
    n = print_diffs(sections, limit=6)
    suggest = {}
    for name, diffs in sections:
        for label, d in diffs:
            offs = [o for o, _, _ in d if isinstance(o, int)]
            if offs:
                suggest.setdefault((label.split()[-1], name), set()).update(offs)
    for (k, name), offs in sorted(suggest.items()):
        cmd, addr = _split_key(k)
        bare = offs - masked_offsets(g.mask, cmd, addr, max(offs) + 1, name)
        print(f"  volatile {name} {k}: {_ranges(offs)}" + (f"  NOT MASKED {_ranges(bare)}" if bare else ""))
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
        sp.add_argument("--mask", default=str(HERE / "mask.json"))
        sp.add_argument("--allow-other-hosts", action="store_true",
                        help="run even if another host is polling (results may then differ)")
        sp.add_argument("--expect-build", help="refuse unless SYS.BUILD_INFO starts with this")
    cap = sub.choices["capture"]
    cap.add_argument("--out", help="reference file to write")
    cap.add_argument("--selfcheck", action="store_true", help="run twice and list volatile bytes")
    cap.add_argument("--provenance", help="JSON file stored under 'provenance' in the reference")
    cmp_ = sub.choices["compare"]
    cmp_.add_argument("--ref", required=True)
    cmp_.add_argument("--force", action="store_true",
                      help="compare even if the script hash or the device IO config differs")
    args = ap.parse_args(argv)
    if args.cmd == "capture" and not args.out and not args.selfcheck:
        ap.error("capture needs --out or --selfcheck")
    try:
        return cmd_capture(args) if args.cmd == "capture" else cmd_compare(args)
    except Disconnected as e:
        print(f"device disconnected: {e}")
        return 4


if __name__ == "__main__":
    sys.exit(main())
