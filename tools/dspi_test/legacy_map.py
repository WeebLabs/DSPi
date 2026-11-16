"""Temporary translation of 1.1.6 vendor requests into paged commands (deleted in plan Phase 6).
Driven by Documentation/protocol/legacy_map.json, whose "_schema" key documents every field. Adapter
kinds: direct, act_value, act_addr, act_packed, split (by wValue selector) and custom (functions here).
Run `python -m tools.dspi_test.legacy_map --check` to validate the map against the plan and registry.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path

from .device import (LEGACY_MAP_REL, REGISTRY_REL, PagedStall, Stall, _find_repo_file,
                     load_legacy_opcodes)

MAX_CONTROL_PAYLOAD = 64
_MAP: dict = {}


def load_map(path=None) -> dict:
    key = str(path) if path else "default"
    if key not in _MAP:
        p = Path(path) if path else _find_repo_file(LEGACY_MAP_REL)
        if p is None:
            raise FileNotFoundError(f"{LEGACY_MAP_REL} not found")
        _MAP[key] = json.loads(p.read_text())
    return _MAP[key]


def _by_code() -> dict[int, tuple[str, dict]]:
    m = load_map()
    if "_by_code" not in m:
        m["_by_code"] = {r["code"]: (n, r) for n, r in m["opcodes"].items()}
    return m["_by_code"]


def target_id(name: str) -> int:
    """Plan numbering of PAGE.NAME from the map's catalog. --check holds it equal to the registry."""
    m = load_map()
    c = m["commands"][name]
    return (m["pages"][c["page"]] << 8) | c["cmd"]


class _Unmapped(Exception):
    """1.1.6 did not handle this request (unknown opcode, direction or selector)."""

    stall = False   # set where an out_stall "raise" object refused it: 1.1.6 STALLed, not ACKed


# --- small pieces of the adapters --------------------------------------------

def _field(spec: dict | None, wvalue: int, default: int) -> int:
    if spec is None:
        return default
    if "const" in spec:
        return spec["const"]
    return ((wvalue >> spec.get("shift", 0)) & spec.get("mask", 0xFFFF)) + spec.get("add", 0)


def _encode(kind: str, v: int) -> bytes:
    if kind == "u8":
        return bytes([v & 0xFF])
    if kind == "u8_sat":
        return bytes([min(v, 0xFF)])
    if kind == "bool":
        return bytes([1 if v else 0])
    raise ValueError(f"unknown encode {kind!r}")


def _status_byte(e: dict, stall: PagedStall) -> int:
    fam = load_map()["status_families"][e["status"]]
    name = stall.result_name
    for table in (e.get("status_overrides", {}), fam["from_vc"]):
        if name in table:
            return table[name]
    raise stall


def _swallow(dev, code, wvalue, why, stall=None):
    log = getattr(dev, "swallowed", None)
    if isinstance(log, list):
        log.append({"opcode": code, "wvalue": wvalue, "why": why,
                    "result": getattr(stall, "result_name", None)})


def _case(e: dict, wvalue: int) -> dict:
    sel = _field(e["selector"], wvalue, wvalue)
    case = e["cases"].get(str(sel), e.get("default"))
    if case is None:
        raise _Unmapped(f"selector {sel} not handled by 1.1.6")
    return case


# --- IN (reads and write-as-reads) --------------------------------------------

def _run_in(dev, code: int, e: dict, wvalue: int, length: int) -> bytes:
    kind = e["kind"]
    if kind == "split":
        return _run_in(dev, code, _case(e, wvalue), wvalue, length)
    if kind == "custom":
        return CUSTOM[e["fn"]](dev, e, wvalue, length)
    tid = target_id(e["target"])
    if kind == "direct":
        try:
            return dev.pget(tid, _field(e.get("addr"), wvalue, wvalue), length)
        except PagedStall as s:
            fill = e.get("on_stall")
            if fill and s.result_name in fill.get("codes", [s.result_name]):
                return bytes([fill["fill"]]) * fill["len"]
            raise
    if kind == "act_value":
        addr, payload = _field(e.get("addr"), wvalue, 0), _encode(e["encode"], _field(e["value"], wvalue, wvalue))
    elif kind == "act_packed":
        addr, payload = _field(e["addr"], wvalue, wvalue), _encode(e["encode"], _field(e["value"], wvalue, wvalue))
    elif kind == "act_addr":
        addr, payload = _field(e.get("addr"), wvalue, wvalue), b""
    else:
        raise ValueError(f"unknown adapter kind {kind!r}")
    try:
        dev.pset(tid, payload, addr)
        status = load_map()["status_families"][e["status"]]["success"]
    except PagedStall as s:
        status = _status_byte(e, s)
    return bytes([status])


def translate_get(dev, code: int, wvalue: int, length: int) -> bytes:
    """An old IN request (read or write-as-read) answered through paged commands."""
    name, rec = _by_code().get(code, (None, {}))
    e = rec.get("in")
    try:
        if e is None:
            raise _Unmapped("no IN handler in 1.1.6")
        return bytes(_run_in(dev, code, e, wvalue, length))[:length]
    except _Unmapped as u:
        raise Stall(code, "GET", None, f"legacy {name or hex(code)}: {u}") from None


# --- OUT (writes) ---------------------------------------------------------------

def _run_out(dev, code: int, e: dict, wvalue: int, data: bytes):
    kind = e["kind"]
    if kind == "split":
        try:
            case = _case(e, wvalue)
        except _Unmapped as u:
            u.stall = e.get("out_stall") == "raise"
            raise
        return _run_out(dev, code, case, wvalue, data)
    if kind == "custom":
        return CUSTOM[e["fn"]](dev, e, wvalue, data)
    if kind != "direct":
        raise ValueError(f"adapter kind {kind!r} cannot carry an OUT write")
    if "trunc" in e:
        data = data[:e["trunc"]]
    dev.pset(target_id(e["target"]), data, _field(e.get("addr"), wvalue, wvalue))


def translate_set(dev, code: int, wvalue: int, data: bytes = b"") -> int:
    """An old OUT write sent as paged SETs. Refusals are swallowed and logged, as 1.1.6 ACKed them."""
    data = bytes(data)
    name, rec = _by_code().get(code, (None, {}))
    e = rec.get("out")
    if e is not None and len(data) > MAX_CONTROL_PAYLOAD and not e.get("bulk"):
        raise Stall(code, "SET", None, "oversized payload (1.1.6 STALLed it at SETUP)")
    try:
        if e is None:
            raise _Unmapped("no OUT handler in 1.1.6")
        _run_out(dev, code, e, wvalue, data)
    except _Unmapped as u:
        if u.stall:
            raise Stall(code, "SET", None, f"legacy {name or hex(code)}: {u}") from None
        _swallow(dev, code, wvalue, str(u))
    except PagedStall as s:
        if e.get("out_stall") == "raise":
            raise
        _swallow(dev, code, wvalue, "paged SET refused", s)
    return len(data)


def golden_transport(dev, direction, code, wvalue, data):
    """golden/capture.py TRANSPORT hook: IN returns the reply bytes, OUT the byte count."""
    if direction in ("IN", "R", "WR"):
        return translate_get(dev, code, wvalue, data)
    return translate_set(dev, code, wvalue, data)


# --- hybrid mode: translate only what the attached firmware implements ------------

def _use(u: str) -> tuple[str, str]:
    name, _, d = u.partition(":")
    return name, d


def _status_uses(e: dict, wvalue: int) -> list[tuple[str, str]]:
    sel = str(wvalue)
    if wvalue in e["levels_raw"] or sel in e["peaks"]:
        return [("MEAS.LEVELS", "G")]
    if sel in e["peak_cpu"]:
        return [("MEAS.LEVELS", "G"), (e["peak_cpu"][sel][1], "G")]
    if sel in e["fields"]:
        return [(e["fields"][sel][0], "G")]
    return []


def required(e: dict, direction: str, wvalue: int) -> list[tuple[str, str]]:
    """(PAGE.NAME, G or S) pairs one old request uses. Raises _Unmapped for a selector 1.1.6 refused."""
    k = e["kind"]
    if k == "split":
        return required(_case(e, wvalue), direction, wvalue)
    if k == "custom":
        return _status_uses(e, wvalue) if e["fn"] == "get_status" else [_use(u) for u in e["uses"]]
    if k == "direct":
        return [(e["target"], "G" if direction == "in" else "S")]
    return [(e["target"], "S")]


def routes_paged(dev, code: int, direction: str, wvalue: int) -> bool:
    """True when every command this old request needs exists on the device (direction "in" or "out")."""
    _, rec = _by_code().get(code, (None, {}))
    e = rec.get(direction)
    if e is None:
        return False
    try:
        need = required(e, direction, wvalue)
    except _Unmapped:
        return False
    if not need:
        return False   # nothing paged to exercise, so the old opcode answers
    have = dev.paged_commands()
    for name, d in need:
        tid = target_id(name)
        gets, sets = have.get(tid >> 8, ((), ()))
        if (tid & 0xFF) not in (gets if d == "G" else sets):
            return False
    return True


def _note(dev, code, translated):
    note = getattr(dev, "note_hybrid", None)
    if note:
        note(_by_code().get(code, (f"0x{code:02X}",))[0], translated)


def hybrid_get(dev, code: int, wvalue: int, length: int) -> bytes:
    if routes_paged(dev, code, "in", wvalue):
        _note(dev, code, True)
        return translate_get(dev, code, wvalue, length)
    _note(dev, code, False)
    return dev.raw_get(code, length, wvalue)


def hybrid_set(dev, code: int, wvalue: int, data: bytes = b"") -> int:
    if routes_paged(dev, code, "out", wvalue):
        _note(dev, code, True)
        return translate_set(dev, code, wvalue, data)
    _note(dev, code, False)
    return dev.raw_set(code, bytes(data), wvalue)


def golden_transport_hybrid(dev, direction, code, wvalue, data):
    """As golden_transport, but requests whose commands the firmware lacks go to page 0 unchanged."""
    if direction in ("IN", "R", "WR"):
        return hybrid_get(dev, code, wvalue, data)
    return hybrid_set(dev, code, wvalue, data)


# --- custom adapters ------------------------------------------------------------

CUSTOM: dict = {}


def custom(name, targets):
    def deco(fn):
        fn.targets = targets
        CUSTOM[name] = fn
        return fn
    return deco


def _get(dev, name, length, addr=0):
    return dev.pget(target_id(name), addr, length)


def _set(dev, name, data=b"", addr=0):
    return dev.pset(target_id(name), data, addr)


_FMT = {"u8": "<B", "u16": "<H", "i16": "<h", "u32": "<I", "i32": "<i"}


@custom("get_status", ["MEAS.LEVELS", "SYS.SYSTEM_STATUS", "SYS.PDM_STATS", "SYS.SPDIF_TX_STATS",
                       "SYS.USB_AUDIO_STATUS", "SYS.PIPELINE_STATS", "INPUT.STATUS", "DIAG.LOOPBACK_STATS"])
def _get_status(dev, e, wvalue, length):
    sel = str(wvalue)
    if wvalue in e["levels_raw"]:
        return _get(dev, "MEAS.LEVELS", length)
    if sel in e["peaks"] or sel in e["peak_cpu"]:
        lv = _get(dev, "MEAS.LEVELS", 64)
        peak = lambda ch: struct.unpack_from("<H", lv, 2 * ch)[0]  # noqa: E731
        if sel in e["peaks"]:
            a, b = e["peaks"][sel]
            return struct.pack("<HH", peak(a), peak(b))
        ch, src, c0, c1 = e["peak_cpu"][sel]
        st = _get(dev, src, 16)
        return struct.pack("<HBB", peak(ch), st[c0], st[c1])
    if sel in e["fields"]:
        src, typ, off = e["fields"][sel]
        try:
            raw = _get(dev, src, 64)
        except PagedStall as s:
            if src in e["zero_if_absent"] and s.result_name == "VC_ERR_UNKNOWN":
                return bytes(4)
            raise
        v = struct.unpack_from(_FMT[typ], raw, off)[0]
        return struct.pack("<i" if v < 0 else "<I", v)
    return bytes(4)


@custom("get_eq_param", ["FILTER.BAND"])
def _get_eq_param(dev, e, wvalue, length):
    ch, band, param = wvalue >> 8, (wvalue >> 3) & 0x1F, wvalue & 0x07
    b = _get(dev, "FILTER.BAND", 16, (ch << 8) | band)
    if param == 0:
        return struct.pack("<I", b[0])
    if param in (1, 2, 3):
        return b[4 * param:4 * param + 4]
    if param == 4:
        return struct.pack("<I", 1 if b[1] == 1 else 0)
    if param == 5:
        return struct.pack("<I", struct.unpack_from("<H", b, 2)[0])
    return bytes(4)


@custom("set_eq_param", ["FILTER.BAND"])
def _set_eq_param(dev, e, wvalue, data):
    if len(data) < 16:
        raise _Unmapped("EqParamPacket shorter than 16 bytes (1.1.6 ignored it)")
    addr = (data[0] << 8) | data[1]
    if len(data) >= 18:
        qp = data[16:18]
    else:
        qp = _get(dev, "FILTER.BAND", 16, addr)[2:4]   # a 16-byte write keeps the stored Linkwitz qp
    _set(dev, "FILTER.BAND", data[2:4] + qp + data[4:16], addr)


@custom("set_matrix_route", ["ROUTE.MATRIX_ROUTE"])
def _set_matrix_route(dev, e, wvalue, data):
    if len(data) < 8:
        raise _Unmapped("MatrixRoutePacket shorter than 8 bytes (1.1.6 ignored it)")
    _set(dev, "ROUTE.MATRIX_ROUTE", data[2:4] + b"\x00\x00" + data[4:8], (data[0] << 8) | data[1])


@custom("get_matrix_route", ["ROUTE.MATRIX_ROUTE"])
def _get_matrix_route(dev, e, wvalue, length):
    r = _get(dev, "ROUTE.MATRIX_ROUTE", 8, wvalue)
    return bytes([(wvalue >> 8) & 0xFF, wvalue & 0xFF]) + r[0:2] + r[4:8]


@custom("get_input_rate", ["SYS.SYSTEM_STATUS", "INPUT.I2S_RX_RATE"])
def _get_input_rate(dev, e, wvalue, length):
    return _get(dev, "SYS.SYSTEM_STATUS", 16)[4:8] + _get(dev, "INPUT.I2S_RX_RATE", 4)[:4]


@custom("preset_get_startup", ["STORE.STARTUP", "STORE.DIRECTORY"])
def _preset_get_startup(dev, e, wvalue, length):
    return _get(dev, "STORE.STARTUP", 2)[:2] + _get(dev, "STORE.DIRECTORY", 7)[4:5]


@custom("cs_display_cfg", ["CS.DISPLAY_CAPS", "CS.DISPLAY_CONFIG"])
def _cs_display_cfg(dev, e, wvalue, length):
    return _get(dev, "CS.DISPLAY_CAPS", 4)[:4] + _get(dev, "CS.DISPLAY_CONFIG", 12)


@custom("clear_clips", ["MEAS.CLIP_FLAGS", "MEAS.CLIP_CLEAR"])
def _clear_clips(dev, e, wvalue, length):
    # Two transfers where 1.1.6 had one, so a clip landing in between is lost.
    flags = _get(dev, "MEAS.CLIP_FLAGS", 4)
    _set(dev, "MEAS.CLIP_CLEAR")
    return flags


@custom("set_preamp_legacy", ["SYS.PLATFORM", "ROUTE.PREAMP"])
def _set_preamp_legacy(dev, e, wvalue, data):
    if len(data) < 4:
        raise _Unmapped("payload shorter than 4 bytes (1.1.6 ignored it)")
    inputs = 8 if _get(dev, "SYS.PLATFORM", 1)[0] == 1 else 2
    for ch in range(inputs):
        _set(dev, "ROUTE.PREAMP", data[:4], ch)


@custom("save_params", ["STORE.PRESET_ACTIVE", "STORE.PRESET_SAVE"])
def _save_params(dev, e, wvalue, length):
    _set(dev, "STORE.PRESET_SAVE", b"", _get(dev, "STORE.PRESET_ACTIVE", 1)[0])
    return b"\x00"


@custom("set_all_params", ["STORE.ALL_PARAMS"])
def _set_all_params(dev, e, wvalue, data):
    if len(data) <= MAX_CONTROL_PAYLOAD:
        raise _Unmapped("bulk SET of 64 bytes or fewer (1.1.6 ACKed and ignored it)")
    if len(data) != e["len"]:
        raise Stall(0xA1, "SET", None, "bulk SET of the wrong length (1.1.6 STALLed it at SETUP)")
    _set(dev, "STORE.ALL_PARAMS", data)   # 1.1.6 ignored wValue here


@custom("usb_error_stats", [])
def _usb_error_stats(dev, e, wvalue, length):
    return bytes(24)


@custom("reset_usb_error_stats", [])
def _reset_usb_error_stats(dev, e, wvalue, length):
    return b"\x01"


# --- consistency check ------------------------------------------------------------

def _walk(e):
    yield e
    if e["kind"] == "split":
        for c in e["cases"].values():
            yield from _walk(c)
        if e.get("default"):
            yield from _walk(e["default"])


def _needs(e: dict, direction: str) -> list[tuple[str, str]]:
    """(target, 'G' or 'S') pairs an adapter relies on."""
    out = []
    for n in _walk(e):
        k = n["kind"]
        if k == "direct":
            out.append((n["target"], "G" if direction == "in" else "S"))
        elif k.startswith("act_"):
            out.append((n["target"], "S"))
        elif k == "custom":
            out += [_use(u) for u in n.get("uses", [])]
    return out


def _plan_commands(plan_text: str) -> dict[str, tuple[int, int]]:
    """{PAGE.NAME: (page, cmd)} parsed from the plan's section 5 tables."""
    out, page, mnem = {}, None, None
    sec = plan_text.split("## 5. New command set", 1)[-1].split("\n## 6.", 1)[0]
    for line in sec.splitlines():
        h = re.match(r"### 5\.\d+ `(\w+)` \((0x[0-9A-Fa-f]+)\)", line)
        if h:
            mnem, page = h.group(1), int(h.group(2), 16)
            continue
        r = re.match(r"\|\s*(0x[0-9A-Fa-f]{2})(?:-(0x[0-9A-Fa-f]{2}))?\s*\|\s*([^|]+)\|", line)
        if not r or page is None:
            continue
        first = int(r.group(1), 16)
        names = re.findall(r"`(\w+)`", r.group(3))
        prefix = names[0].rsplit("_", 1)[0] if names else ""
        for i, n in enumerate(names):
            out[f"{mnem}.{n if i == 0 else prefix + n}"] = (page, first + i)
    return out


def _old_dirs(commands_md: str) -> dict[str, str]:
    """{NAME: Dir} from commands.md section 21, the independent record of 1.1.6 directions."""
    sec = commands_md.split("## 21. Complete opcode list", 1)
    if len(sec) < 2:
        return {}
    out = {}
    for m in re.finditer(r"^\| 0x[0-9A-F]{2} \| (\w+) \| ([\w+-]+) \|", sec[1].split("\n## 22.", 1)[0], re.M):
        out[m.group(1)] = {"W": "W", "R": "R", "W-as-R": "WR", "W+R": "W+R"}.get(m.group(2), m.group(2))
    return out


def check(registry_path=None, verbose=False) -> tuple[list[str], list[str]]:
    """Returns (errors, notes). Missing registry pages are notes, not errors."""
    errors, notes = [], []
    m = load_map()
    legacy = load_legacy_opcodes()
    cat, pages = m["commands"], m["pages"]

    if len(legacy) != 246:
        errors.append(f"golden/legacy_opcodes.json has {len(legacy)} opcodes, expected 246")
    expect_dirs = {"W": {"out"}, "R": {"in"}, "WR": {"in"}, "W+R": {"in", "out"}}
    md = _find_repo_file(Path("Documentation") / "commands.md")
    old_dirs = _old_dirs(md.read_text()) if md else {}
    if not old_dirs:
        notes.append("commands.md section 21 not found: old directions taken from the map only")
    for name, code in sorted(legacy.items(), key=lambda kv: kv[1]):
        rec = m["opcodes"].get(name)
        if rec is None:
            errors.append(f"{name} (0x{code:02X}): no mapping")
            continue
        if rec["code"] != code:
            errors.append(f"{name}: code 0x{rec['code']:02X} != legacy 0x{code:02X}")
        od = old_dirs.get(name, rec["old_dir"])
        if od != rec["old_dir"]:
            errors.append(f"{name}: old_dir {rec['old_dir']} but commands.md says {od}")
        have = {d for d in ("in", "out") if d in rec}
        if have != expect_dirs.get(od, set()):
            errors.append(f"{name}: maps {sorted(have)} but 1.1.6 direction is {od}")
    for name in set(m["opcodes"]) - set(legacy):
        errors.append(f"{name}: mapped but not a 1.1.6 opcode")

    plan = _find_repo_file(Path("Documentation") / "Features" / "vendor_command_overhaul_plan.md")
    if plan:
        pc = _plan_commands(plan.read_text())
        for n, c in cat.items():
            want = (pages[c["page"]], c["cmd"])
            if pc.get(n) != want:
                errors.append(f"catalog {n} = {want[0]:#04x}/{want[1]:#04x}, plan section 5 says {pc.get(n)}")
        for n in sorted(set(pc) - set(cat)):
            errors.append(f"plan section 5 command {n} missing from the catalog")
    else:
        notes.append("plan not found: catalog numbering not cross-checked")

    used = set()
    for name, rec in m["opcodes"].items():
        for d in ("in", "out"):
            if d not in rec:
                continue
            for t, need in _needs(rec[d], d):
                used.add(t)
                c = cat.get(t)
                if c is None:
                    errors.append(f"{name}.{d}: target {t} not in the catalog")
                elif need and need not in c["dir"]:
                    errors.append(f"{name}.{d}: target {t} has no {'GET' if need == 'G' else 'SET'}")
            if rec[d]["kind"] == "split":
                for sel, n in rec[d]["cases"].items():
                    c = cat.get(n.get("target"), {})
                    if c.get("addr") == "NONE" and "addr" not in n and n["kind"] != "act_value" and sel != "0":
                        errors.append(f"{name}.{d}: selector {sel} would reach {n['target']} as its address")
            for n in _walk(rec[d]):
                if n["kind"] == "custom":
                    fn = CUSTOM.get(n["fn"])
                    if fn is None:
                        errors.append(f"{name}.{d}: custom {n['fn']} not defined")
                    elif not isinstance(n.get("uses"), list):
                        errors.append(f"{name}.{d}: custom {n['fn']} has no uses list")
                    elif set(fn.targets) != {_use(u)[0] for u in n["uses"]}:
                        errors.append(f"{name}.{d}: custom {n['fn']} uses differ from the code")
                    for u in n.get("uses", []):
                        if not re.fullmatch(r"\w+\.\w+:[GS]", u):
                            errors.append(f"{name}.{d}: malformed use {u!r} (want PAGE.NAME:G or :S)")
                if n.get("status") and n["status"] not in m["status_families"]:
                    errors.append(f"{name}.{d}: unknown status family {n['status']}")

    rp = Path(registry_path) if registry_path else _find_repo_file(REGISTRY_REL)
    reg = json.loads(rp.read_text()) if rp and rp.is_file() else {"pages": []}
    regpages = {p["mnemonic"]: p for p in reg.get("pages", [])}
    pending: dict[str, int] = {}
    for mnem, rpg in regpages.items():
        if mnem not in pages or rpg["page"] != pages[mnem]:
            errors.append(f"registry page {mnem} {rpg['page']:#04x} is not {mnem} in the map")
        for x in rpg.get("commands", []):
            if f"{mnem}.{x['name']}" not in cat:
                notes.append(f"registry {mnem}.{x['name']} has no catalog entry (added after plan section 5?)")
    for t in sorted(cat):
        c = cat[t]
        mnem, cname = t.split(".", 1)
        rpg = regpages.get(mnem)
        rc = next((x for x in rpg["commands"] if x["name"] == cname), None) if rpg else None
        if rc is None:
            pending[mnem] = pending.get(mnem, 0) + 1
            continue
        if rc["cmd"] != c["cmd"]:
            errors.append(f"{t}: registry cmd {rc['cmd']:#04x}, map {c['cmd']:#04x}")
        if ("G" in c["dir"]) != bool(rc.get("get")) or ("S" in c["dir"]) != bool(rc.get("set")):
            errors.append(f"{t}: registry get/set {rc.get('get')}/{rc.get('set')}, map dir {c['dir']}")
        if (c["addr"] == "NONE") != (rc.get("addr") == "NONE"):
            errors.append(f"{t}: registry addr {rc.get('addr')}, map {c['addr']}")
        gl, rgl = c.get("get_len"), rc.get("get_len")
        if isinstance(gl, int) and isinstance(rgl, int) and gl != rgl:
            errors.append(f"{t}: registry get_len {rgl}, map {gl}")
        s = c.get("set")
        if s and rc.get("set") and "BULK" not in rc.get("flags", []):
            if not (rc.get("set_min", 0) <= s[0] and s[1] <= rc.get("set_max", 0)):
                errors.append(f"{t}: registry SET {rc.get('set_min')}-{rc.get('set_max')} rejects map {s}")
    for mn, n in sorted(pending.items(), key=lambda kv: pages[kv[0]]):
        notes.append(f"page {mn}: {n} command(s) not yet in registry")
    return errors, notes


def _summary() -> str:
    m = load_map()
    kinds: dict[str, int] = {}
    for rec in m["opcodes"].values():
        for d in ("in", "out"):
            for n in _walk(rec[d]) if d in rec else ():
                kinds[n["kind"]] = kinds.get(n["kind"], 0) + 1
    return (f"{len(m['opcodes'])} opcodes, {len(m['commands'])} catalog commands, adapters "
            + ", ".join(f"{k} {v}" for k, v in sorted(kinds.items())))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="legacy_map")
    ap.add_argument("--check", action="store_true", help="validate legacy_map.json")
    ap.add_argument("--registry", help="dspi_commands.json to check against (default: the repo copy)")
    args = ap.parse_args(argv)
    if not args.check:
        ap.print_help()
        return 0
    errors, notes = check(args.registry)
    print(_summary())
    for n in notes:
        print("note:", n)
    for e in errors:
        print("ERROR:", e)
    print("legacy_map check:", "FAIL" if errors else "OK", f"({len(errors)} errors, {len(notes)} notes)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
