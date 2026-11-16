"""
Native paged tests for the Control Surfaces page (plan section 5.10). CS.REVERT is only sent in
shapes the dispatcher refuses (a revert drops stored non-button bindings). Preview writes use empty
records and claim no GPIO; CS.SAVE runs only once every record matches the snapshot.
"""

import struct
import time

from ..device import ProtocolError, Stall
from ..framework import Skip, test
from .paged_sys import _expect_stall, _registry

CS = 0x0A
PENDING = 0x16
INVALID_AUX = 0x26
NO_IR = 0x1E
TYPE_IR = 7
AUX_TYPES = (9, 10)

# Known-valid records from the golden script (tools/dspi_test/golden/script.py).
IR_REC = struct.pack("<BBBBBBhhBBI", 3, 4, 0, 0, 0, 1, 0, 0, 0, 0, 0x00FF12ED)
GROUP_REC = struct.pack("<BBBBI32s", 2, 0, 0, 0, 0x05, b"PagedGrp")
STEP0 = struct.pack("<BBBBBBhhH", 3, 5, 0, 0, 0, 0, 1, 0, 0)
STEP1 = struct.pack("<BBBBBBhhH", 9, 4, 0, 0, 0, 0, 0, 0, 10)
PAGE_REC = bytes([17, 1, 0, 0x09])


def _paged(dev):
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    if CS not in dev.paged_commands():
        raise Skip("CS page not implemented on this firmware")


def _limits(dev):
    caps = dev.pget("CS.CAPS")
    t = 4 + 4 * caps[2]
    return {"slots": caps[1], "nouns": caps[3], "ir": caps[t], "groups": caps[t + 1],
            "macros": caps[t + 2], "steps": caps[t + 3], "pages": dev.pget("CS.DISPLAY_CAPS")[0]}


def _status(dev):
    s = dev.pget("CS.STATUS")
    return {"last": s[0], "slot": s[1], "dirty": s[3], "raw": s}


def _settle(dev, timeout_s=3.0):
    """CS.STATUS once the last deferred request has left PENDING, else None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            st = _status(dev)
        except (Stall, OSError):
            dev.wait_ready()
            continue
        if st["last"] != PENDING:
            return st
        time.sleep(0.01)
    return None


def _snapshot(dev, lim):
    def rng(cmd, n, addr=lambda i: i):
        return [dev.pget(cmd, addr=addr(i)) for i in range(n)]
    return {"binding": rng("CS.BINDING", lim["slots"]), "name": rng("CS.SLOT_NAME", lim["slots"]),
            "ir": rng("CS.IR_CMD", lim["ir"]), "group": rng("CS.GROUP", lim["groups"]),
            "macro": rng("CS.MACRO", lim["macros"]), "page": rng("CS.DISPLAY_PAGE", lim["pages"]),
            "display": dev.pget("CS.DISPLAY_CONFIG")}


def _name_payload(raw32):
    name = bytes(raw32).split(b"\0")[0]
    return name if name else b"\0"


def _put_record(dev, name, payload, addr, tag):
    """Preview SET; None once CS.STATUS reports it applied, else what went wrong."""
    try:
        dev.pset(name, payload, addr=addr)
    except Stall as e:
        return f"{name} @0x{addr:04X} refused ({e})"
    st = _settle(dev)
    if st is None or (st["last"], st["slot"]) != (0, tag):
        return f"{name} @0x{addr:04X} not applied: {st and (hex(st['last']), hex(st['slot']))}"
    return None


def _snapshot_diff(snap, after):
    out = []
    for k, v in snap.items():
        if isinstance(v, list):
            out += [f"{k}[{i}]" for i, (a, b) in enumerate(zip(v, after[k])) if a != b]
        elif v != after[k]:
            out.append(k)
    return out


def restore_preview(dev, lim, snap):
    """Rewrite every CS record that differs from `snap`, then CS.SAVE so the preview is clean.
    Raises AssertionError naming what is left (SAVE is then never sent); returns the final CS.STATUS."""
    errors = []

    def fix(name, cur, want, addr, tag, payload=None):
        if cur != want:
            e = _put_record(dev, name, want if payload is None else payload, addr, tag)
            if e:
                errors.append(e)
    for i in range(lim["slots"]):
        fix("CS.BINDING", dev.pget("CS.BINDING", addr=i), snap["binding"][i], i, i)
        fix("CS.SLOT_NAME", dev.pget("CS.SLOT_NAME", addr=i), snap["name"][i], i, i, _name_payload(snap["name"][i]))
    for i in range(lim["ir"]):
        fix("CS.IR_CMD", dev.pget("CS.IR_CMD", addr=i), snap["ir"][i], i, 0x80 | i)
    for g in range(lim["groups"]):
        fix("CS.GROUP", dev.pget("CS.GROUP", addr=g), snap["group"][g], g, 0x40 | g)
    for m in range(lim["macros"]):
        if dev.pget("CS.MACRO", addr=m) == snap["macro"][m]:
            continue
        fix("CS.MACRO_HEADER", dev.pget("CS.MACRO_HEADER", addr=m), snap["macro"][m][:33] + bytes(3), m, 0x60 | m)
        for k in range(lim["steps"]):
            a = (k << 8) | m
            fix("CS.MACRO_STEP", dev.pget("CS.MACRO_STEP", addr=a), snap["macro"][m][36 + 12 * k:48 + 12 * k],
                a, 0x60 | m)
    for p in range(lim["pages"]):
        fix("CS.DISPLAY_PAGE", dev.pget("CS.DISPLAY_PAGE", addr=p), snap["page"][p], p, 0x50 | p)
    fix("CS.DISPLAY_CONFIG", dev.pget("CS.DISPLAY_CONFIG"), snap["display"], 0, 0x50)
    left = _snapshot_diff(snap, _snapshot(dev, lim))
    if left:
        raise AssertionError(f"CS preview left dirty, CS.SAVE not sent: {left} differ from the pre-test snapshot"
                             + (f"; restore errors: {errors}" if errors else ""))
    st = _status(dev)
    if st["dirty"]:
        dev.pset("CS.SAVE")
        time.sleep(0.3)
        dev.wait_ready()
        st = _settle(dev, timeout_s=5.0)
        if st is None or st["dirty"]:
            raise AssertionError(f"CS.SAVE of the restored records left the preview dirty: {st and st['raw'].hex()}")
    return st


@test("paged_cs")
def pcs_command_info_matches_registry(dev, profile, chk):
    """SYS.COMMAND_INFO and PAGE_COMMANDS for every CS command match the generated registry."""
    _paged(dev)
    page = next((p for p in _registry(dev)["pages"] if p["page"] == CS), None)
    if page is None:
        raise Skip("registry has no CS page")
    gets, sets = dev.paged_commands()[CS]
    for c in page["commands"]:
        chk.eq(c["cmd"] in gets, bool(c["get"]), f"GET bit of CS.{c['name']}")
        chk.eq(c["cmd"] in sets, bool(c["set"]), f"SET bit of CS.{c['name']}")
        r = dev.pget(0x0113, addr=(CS << 8) | c["cmd"], length=8)
        flags, get_len, smin, smax, _akind, has = struct.unpack("<HHBBBB", r)
        want = 0xFFFF if c["get_len"] == "var" else (c["get_len"] or 0)
        chk.eq(get_len, want if c["get"] else 0, f"CS.{c['name']} get_len")
        chk.eq((smin, smax), (c["set_min"], c["set_max"]), f"CS.{c['name']} SET bounds")
        chk.eq(has & 3, (1 if c["get"] else 0) | (2 if c["set"] else 0), f"CS.{c['name']} directions")
    chk.eq(sorted((gets | sets) - {c["cmd"] for c in page["commands"]}), [], "no unlisted CS command")


@test("paged_cs")
def pcs_reads_shape(dev, profile, chk):
    """Every CS GET has its table size; MACRO agrees with MACRO_HEADER + MACRO_STEP, AUX_ALL with AUX_STATE/LEVEL."""
    _paged(dev)
    lim = _limits(dev)
    chk.eq(len(dev.pget("CS.CAPS")), 52, "CAPS length")
    for n in range(lim["nouns"]):
        d = dev.pget("CS.NOUN_DESC", addr=n)
        if chk.eq(len(d), 12, f"NOUN_DESC {n} length"):
            lo, hi = struct.unpack_from("<hh", d, 4)
            chk.ok(lo <= hi, f"NOUN_DESC {n} min_q {lo} <= max_q {hi}")
    st = dev.pget("CS.STATUS")
    chk.eq(len(st), 41, "STATUS length")
    chk.member(st[3], (0, 1), "STATUS dirty flag")
    chk.eq(len(dev.pget("CS.EXT_STATUS")), 24, "EXT_STATUS length")
    chk.ok(len(dev.pget("CS.IR_LEARN_STATUS")) > 0, "IR_LEARN_STATUS answers")
    aux_all = dev.pget("CS.AUX_ALL")
    chk.eq(len(aux_all), 48, "AUX_ALL length")
    for s in range(lim["slots"]):
        b = dev.pget("CS.BINDING", addr=s)
        chk.eq(len(b), 24, f"BINDING {s} length")
        nm = dev.pget("CS.SLOT_NAME", addr=s)
        chk.ok(len(nm) == 32 and b"\0" in nm, f"SLOT_NAME {s} is 32 NUL-terminated bytes: {nm!r}")
        if b[0] in AUX_TYPES:
            chk.eq(dev.pget("CS.AUX_STATE", addr=s), aux_all[s:s + 1], f"AUX_STATE {s} == AUX_ALL")
            chk.eq(dev.pget("CS.AUX_LEVEL", addr=s), aux_all[16 + 2 * s:18 + 2 * s], f"AUX_LEVEL {s} == AUX_ALL")
        else:
            _expect_stall(chk, lambda: dev.pget("CS.AUX_STATE", addr=s), "VC_ERR_STATE", f"AUX_STATE {s}, not an aux")
            _expect_stall(chk, lambda: dev.pget("CS.AUX_LEVEL", addr=s), "VC_ERR_STATE", f"AUX_LEVEL {s}, not an aux")
    for i in range(lim["ir"]):
        chk.eq(len(dev.pget("CS.IR_CMD", addr=i)), 16, f"IR_CMD {i} length")
    for g in range(lim["groups"]):
        chk.eq(len(dev.pget("CS.GROUP", addr=g)), 40, f"GROUP {g} length")
    for m in range(lim["macros"]):
        mac = dev.pget("CS.MACRO", addr=m)
        chk.eq(len(mac), 132, f"MACRO {m} length")
        chk.eq(dev.pget("CS.MACRO_HEADER", addr=m), mac[:33] + bytes(3), f"MACRO_HEADER {m} == header, reserved 0")
        for k in range(lim["steps"]):
            chk.eq(dev.pget("CS.MACRO_STEP", addr=(k << 8) | m), mac[36 + 12 * k:48 + 12 * k],
                   f"MACRO_STEP {k} of macro {m}")
    caps = dev.pget("CS.DISPLAY_CAPS")
    chk.eq(len(caps), 4, "DISPLAY_CAPS length")
    chk.eq(caps[2:], b"\0\0", "DISPLAY_CAPS reserved")
    cfg = dev.pget("CS.DISPLAY_CONFIG")
    chk.eq(len(cfg), 12, "DISPLAY_CONFIG length")
    chk.ok(cfg[1] < caps[0], f"DISPLAY_CONFIG home page {cfg[1]} below the page count {caps[0]}")
    chk.eq(cfg[10:], b"\0\0", "DISPLAY_CONFIG reserved")
    for p in range(lim["pages"]):
        chk.eq(len(dev.pget("CS.DISPLAY_PAGE", addr=p)), 4, f"DISPLAY_PAGE {p} length")
    ds = dev.pget("CS.DISPLAY_STATUS")
    chk.eq(len(ds), 8, "DISPLAY_STATUS length")
    chk.in_range(ds[0], 0, 3, "display init state")
    chk.ok(ds[1] < caps[0] or ds[1] == 0xFF, f"display current page {ds[1]}")
    chk.ok(ds[3] < caps[1], f"display model {ds[3]} below the model count {caps[1]}")
    chk.eq(ds[6:], b"\0\0", "DISPLAY_STATUS reserved")


@test("paged_cs")
def pcs_address_errors(dev, profile, chk):
    """Out-of-range addresses STALL with VC_ERR_ADDRESS before any handler runs; CS.STATUS is untouched."""
    _paged(dev)
    lim = _limits(dev)
    before = _status(dev)["raw"]
    bad_get = [("CS.NOUN_DESC", lim["nouns"]), ("CS.BINDING", lim["slots"]), ("CS.SLOT_NAME", lim["slots"]),
               ("CS.SLOT_NAME", 0x0100), ("CS.IR_CMD", lim["ir"]), ("CS.GROUP", lim["groups"]),
               ("CS.MACRO", lim["macros"]), ("CS.MACRO_HEADER", lim["macros"]),
               ("CS.MACRO_STEP", lim["steps"] << 8), ("CS.MACRO_STEP", lim["macros"]),
               ("CS.AUX_STATE", lim["slots"]), ("CS.AUX_LEVEL", lim["slots"]), ("CS.DISPLAY_PAGE", lim["pages"])]
    bad_get += [(n, 1) for n in ("CS.CAPS", "CS.STATUS", "CS.EXT_STATUS", "CS.IR_LEARN_STATUS", "CS.AUX_ALL",
                                 "CS.DISPLAY_CONFIG", "CS.DISPLAY_CAPS", "CS.DISPLAY_STATUS")]
    for name, a in bad_get:
        _expect_stall(chk, lambda: dev.pget(name, addr=a), "VC_ERR_ADDRESS", f"GET {name} @0x{a:04X}")
    bad_set = [("CS.BINDING", lim["slots"], 24), ("CS.SLOT_NAME", lim["slots"], 4), ("CS.IR_CMD", lim["ir"], 16),
               ("CS.GROUP", lim["groups"], 40), ("CS.MACRO_HEADER", lim["macros"], 36),
               ("CS.MACRO_STEP", lim["steps"] << 8, 12), ("CS.MACRO_FIRE", lim["macros"], 0),
               ("CS.AUX_STATE", lim["slots"], 1), ("CS.AUX_LEVEL", lim["slots"], 2),
               ("CS.DISPLAY_PAGE", lim["pages"], 4), ("CS.DISPLAY_CONFIG", 1, 12),
               ("CS.MACRO_CANCEL", 1, 0), ("CS.IR_LEARN_CANCEL", 1, 0)]
    for name, a, n in bad_set:
        _expect_stall(chk, lambda: dev.pset(name, bytes(n), addr=a), "VC_ERR_ADDRESS", f"SET {name} @0x{a:04X}")
    chk.eq(_status(dev)["raw"], before, "CS.STATUS unchanged by refused requests")


@test("paged_cs")
def pcs_length_and_direction(dev, profile, chk):
    """Payloads outside each SET's bounds give VC_ERR_LENGTH, the wrong direction VC_ERR_DIRECTION."""
    _paged(dev)
    before = _status(dev)["raw"]
    bad_len = [("CS.BINDING", 23), ("CS.BINDING", 25), ("CS.SLOT_NAME", 0), ("CS.SLOT_NAME", 33),
               ("CS.IR_CMD", 15), ("CS.IR_CMD", 17), ("CS.GROUP", 39), ("CS.MACRO_HEADER", 35),
               ("CS.MACRO_STEP", 11), ("CS.MACRO_STEP", 13), ("CS.AUX_STATE", 0), ("CS.AUX_STATE", 2),
               ("CS.AUX_LEVEL", 1), ("CS.DISPLAY_CONFIG", 11), ("CS.DISPLAY_CONFIG", 13),
               ("CS.DISPLAY_PAGE", 3), ("CS.SAVE", 1), ("CS.REVERT", 1), ("CS.MACRO_FIRE", 1),
               ("CS.MACRO_CANCEL", 1), ("CS.IR_LEARN_ARM", 1), ("CS.IR_LEARN_CANCEL", 1)]
    for name, n in bad_len:
        _expect_stall(chk, lambda: dev.pset(name, bytes(n)), "VC_ERR_LENGTH", f"{n}-byte SET {name}")
    for name in ("CS.SAVE", "CS.REVERT", "CS.MACRO_FIRE", "CS.MACRO_CANCEL", "CS.IR_LEARN_ARM",
                 "CS.IR_LEARN_CANCEL"):
        _expect_stall(chk, lambda: dev.pget(name, length=8), "VC_ERR_DIRECTION", f"GET {name}")
    for name in ("CS.CAPS", "CS.NOUN_DESC", "CS.STATUS", "CS.EXT_STATUS", "CS.IR_LEARN_STATUS", "CS.MACRO",
                 "CS.AUX_ALL", "CS.DISPLAY_CAPS", "CS.DISPLAY_STATUS"):
        _expect_stall(chk, lambda: dev.pset(name), "VC_ERR_DIRECTION", f"SET {name}")
    chk.eq(_status(dev)["raw"], before, "CS.STATUS unchanged by refused requests")


@test("paged_cs")
def pcs_ir_learn_and_macro_actions(dev, profile, chk):
    """IR_LEARN_CANCEL while idle, MACRO_FIRE of an empty macro and MACRO_CANCEL succeed as no-ops, twice over."""
    _paged(dev)
    lim = _limits(dev)
    if dev.pget("CS.IR_LEARN_STATUS")[0] != 0:
        raise Skip("IR learn is not idle")
    ext = dev.pget("CS.EXT_STATUS")
    if ext[3] != 0xFF:
        raise Skip("a macro is running")
    empty = [m for m in range(lim["macros"]) if dev.pget("CS.MACRO", addr=m)[32] == 0]
    if not empty:
        raise Skip("no macro with zero steps")
    before = _status(dev)["raw"]
    for k in (1, 2):
        chk.eq(dev.pset("CS.IR_LEARN_CANCEL"), 0, f"IR_LEARN_CANCEL accepted ({k})")
        chk.eq(dev.pget("CS.IR_LEARN_STATUS")[0], 0, f"learn still idle ({k})")
    m = empty[-1]
    chk.eq(dev.pset("CS.MACRO_FIRE", addr=m), 0, f"MACRO_FIRE of empty macro {m}")
    chk.eq(dev.pget("CS.EXT_STATUS")[3], 0xFF, "an empty macro never runs")
    chk.eq(dev.pset("CS.MACRO_CANCEL"), 0, "MACRO_CANCEL while idle")
    chk.eq(dev.pset("CS.MACRO_FIRE", addr=m), 0, f"MACRO_FIRE of empty macro {m} again")
    chk.eq(dev.pset("CS.MACRO_CANCEL"), 0, "MACRO_CANCEL again")
    chk.eq(dev.pget("CS.EXT_STATUS"), ext, "EXT_STATUS unchanged")
    chk.eq(_status(dev)["raw"], before, "CS.STATUS unchanged")


@test("paged_cs", mutating=True)
def pcs_rejections_report_status(dev, profile, chk):
    """AUX SETs on a non-aux slot and IR_LEARN_ARM without a receiver STALL with VC_ERR_STATE and post the reason.

    Leaves CS.STATUS last_status/last_slot on the last rejection (runtime only, nothing stored);
    pcs_preview_roundtrip_and_save ends on a SAVE result when flash tests run.
    """
    _paged(dev)
    lim = _limits(dev)
    types = [dev.pget("CS.BINDING", addr=s)[0] for s in range(lim["slots"])]
    free = [s for s in range(lim["slots"]) if types[s] == 0]
    if not free:
        raise Skip("no empty binding slot")
    s = free[-1]
    for name, payload in (("CS.AUX_STATE", b"\x00"), ("CS.AUX_LEVEL", b"\x00\x00")):
        _expect_stall(chk, lambda: dev.pset(name, payload, addr=s), "VC_ERR_STATE", f"{name} on empty slot {s}")
        st = _status(dev)
        chk.eq((st["last"], st["slot"]), (INVALID_AUX, s), f"{name} posts INVALID_AUX for slot {s}")
    if len(free) > 1:
        s2 = free[-2]   # another empty slot, so the post visibly names the new slot
        _expect_stall(chk, lambda: dev.pset("CS.AUX_STATE", b"\x01", addr=s2), "VC_ERR_STATE",
                      f"CS.AUX_STATE on empty slot {s2}")
        st = _status(dev)
        chk.eq((st["last"], st["slot"]), (INVALID_AUX, s2), f"the post follows the latest slot {s2}")
    if TYPE_IR in types:
        chk.note("an IR receiver binding exists: IR_LEARN_ARM not sent")
        return
    _expect_stall(chk, lambda: dev.pset("CS.IR_LEARN_ARM"), "VC_ERR_STATE", "IR_LEARN_ARM without a receiver")
    st = _status(dev)
    chk.eq((st["last"], st["slot"]), (NO_IR, 0xFF), "IR_LEARN_ARM posts NO_IR")
    chk.eq(dev.pget("CS.IR_LEARN_STATUS")[0], 0, "learn still idle")


@test("paged_cs", mutating=True, flash=1)
def pcs_preview_roundtrip_and_save(dev, profile, chk):
    """Preview SETs on empty records round-trip, are restored in every outcome, then CS.SAVE of identical content clears dirty."""
    _paged(dev)
    lim = _limits(dev)
    st0 = _status(dev)
    if st0["dirty"]:
        raise Skip("Control Surfaces has an unsaved preview")
    if st0["last"] == PENDING:
        raise Skip("a deferred CS request is still pending")
    snap = _snapshot(dev, lim)
    if any(b[0] in AUX_TYPES for b in snap["binding"]):
        raise Skip("an aux binding exists: SAVE would fold its live value")

    def last_free(records, size):
        idx = [i for i, r in enumerate(records) if r == bytes(size)]
        if not idx:
            raise Skip("no empty record of every kind")
        return idx[-1]
    slot = last_free(snap["binding"], 24)
    sub, grp, mac, page = (last_free(snap["ir"], 16), last_free(snap["group"], 40),
                           last_free(snap["macro"], 132), last_free(snap["page"], 4))

    def put(name, payload, addr, tag):
        err = _put_record(dev, name, payload, addr, tag)
        chk.ok(err is None, err or "")

    try:
        rec = bytes([0, 17, 0, 0, 0, 0, 0, 1]) + bytes(16)   # CS_TYPE_NONE: stored, never claims a pin
        put("CS.BINDING", rec, slot, slot)
        chk.eq(dev.pget("CS.BINDING", addr=slot), rec, "BINDING round trip")
        rec2 = bytes([0, 18]) + rec[2:]
        put("CS.BINDING", rec2, slot, slot)
        chk.eq(dev.pget("CS.BINDING", addr=slot), rec2, "a second BINDING write replaces the record")

        put("CS.SLOT_NAME", b"PagedName", slot, slot)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"PagedName".ljust(32, b"\0"), "SLOT_NAME round trip")
        put("CS.SLOT_NAME", b"A" * 32, slot, slot)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"A" * 31 + b"\0", "32-byte name keeps a NUL terminator")
        put("CS.SLOT_NAME", b"Short", slot, slot)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"Short".ljust(32, b"\0"), "a shorter name clears the old tail")

        put("CS.IR_CMD", IR_REC, sub, 0x80 | sub)
        chk.eq(dev.pget("CS.IR_CMD", addr=sub), IR_REC, "IR_CMD round trip")

        put("CS.GROUP", GROUP_REC, grp, 0x40 | grp)
        chk.eq(dev.pget("CS.GROUP", addr=grp), GROUP_REC, "GROUP round trip")
        grp2 = GROUP_REC[:8] + b"OtherGrp".ljust(32, b"\0")
        put("CS.GROUP", grp2, grp, 0x40 | grp)
        chk.eq(dev.pget("CS.GROUP", addr=grp), grp2, "a second GROUP write replaces the record")

        put("CS.MACRO_STEP", STEP0, mac, 0x60 | mac)
        chk.eq(dev.pget("CS.MACRO_STEP", addr=mac), STEP0, "MACRO_STEP 0 round trip")
        put("CS.MACRO_STEP", STEP1, (1 << 8) | mac, 0x60 | mac)
        chk.eq(dev.pget("CS.MACRO_STEP", addr=(1 << 8) | mac), STEP1, "MACRO_STEP 1 round trip")
        hdr = struct.pack("<32sBBBB", b"PagedMacro", 0, 0, 0, 0)
        put("CS.MACRO_HEADER", hdr, mac, 0x60 | mac)
        chk.eq(dev.pget("CS.MACRO_HEADER", addr=mac), hdr, "MACRO_HEADER round trip")
        m = dev.pget("CS.MACRO", addr=mac)
        chk.eq((m[:36], m[36:48], m[48:60], m[60:]), (hdr, STEP0, STEP1, bytes(72)),
               "MACRO is MACRO_HEADER followed by every MACRO_STEP")

        put("CS.DISPLAY_PAGE", PAGE_REC, page, 0x50 | page)
        chk.eq(dev.pget("CS.DISPLAY_PAGE", addr=page), PAGE_REC, "DISPLAY_PAGE round trip")

        cfg = bytearray(snap["display"])
        cfg[6] ^= 1                                   # brightness, one step
        put("CS.DISPLAY_CONFIG", bytes(cfg), 0, 0x50)
        chk.eq(dev.pget("CS.DISPLAY_CONFIG"), bytes(cfg), "DISPLAY_CONFIG round trip")
        chk.eq(dev.pget("CS.DISPLAY_CAPS")[0], lim["pages"], "DISPLAY_CAPS unchanged by a config write")
        chk.eq(_status(dev)["dirty"], 1, "the preview is marked dirty")
    finally:
        st = restore_preview(dev, lim, snap)
    chk.ok((st["last"], st["slot"], st["dirty"]) == (0, 0xFF, 0),
           f"SAVE completes clean: {(hex(st['last']), hex(st['slot']), st['dirty'])}")
    chk.eq(_snapshot(dev, lim), snap, "records unchanged by the SAVE")
