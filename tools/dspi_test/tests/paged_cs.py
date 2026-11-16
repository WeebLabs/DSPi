"""
Native paged tests for the Control Surfaces page (plan section 5.10). CS.REVERT is only sent in
shapes the dispatcher refuses (a 1.1.6 revert drops stored non-button bindings). Preview writes
use empty records and claim no GPIO; CS.SAVE runs only once every record matches the snapshot.
"""

import struct
import time

from ..device import OP, PagedStall, ProtocolError, Stall
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


def _same(chk, new_fn, old_fn, msg, tries=3):
    """New and old reads equal; retried because live status bytes may move between reads."""
    for _ in range(tries):
        try:
            b = old_fn()
        except Stall:
            chk.note(f"{msg}: page 0 no longer answers; comparison skipped")
            return True
        a = new_fn()
        if a == b:
            return chk.ok(True, msg)
    return chk.eq(a, b, msg)


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
def pcs_reads_match_legacy(dev, profile, chk):
    """Every CS GET is byte-identical to its 1.1.6 opcode, including each former selector."""
    _paged(dev)
    lim = _limits(dev)
    _same(chk, lambda: dev.pget("CS.CAPS"), lambda: dev.raw_get(OP.GET_CS_CAPS, 52, 0xFFFF), "CAPS == 0x86 @0xFFFF")
    for n in range(lim["nouns"]):
        _same(chk, lambda: dev.pget("CS.NOUN_DESC", addr=n), lambda: dev.raw_get(OP.GET_CS_CAPS, 12, n),
              f"NOUN_DESC {n} == 0x86 @{n}")
    _same(chk, lambda: dev.pget("CS.STATUS"), lambda: dev.raw_get(OP.GET_CS_STATUS, 41), "STATUS == 0x87")
    _same(chk, lambda: dev.pget("CS.EXT_STATUS"), lambda: dev.raw_get(OP.GET_CS_EXT_STATUS, 24),
          "EXT_STATUS == 0x26")
    _same(chk, lambda: dev.pget("CS.IR_LEARN_STATUS"), lambda: dev.raw_get(OP.CS_IR_LEARN, 8, 2),
          "IR_LEARN_STATUS == 0x8F @2")
    for s in range(lim["slots"]):
        _same(chk, lambda: dev.pget("CS.BINDING", addr=s), lambda: dev.raw_get(OP.GET_CS_BINDING, 24, s),
              f"BINDING {s}")
        _same(chk, lambda: dev.pget("CS.SLOT_NAME", addr=s), lambda: dev.raw_get(OP.GET_CS_NAME, 32, s),
              f"SLOT_NAME {s}")
        try:
            old = dev.raw_get(OP.GET_CS_AUX_STATE, 1, s), dev.raw_get(OP.GET_CS_AUX_LEVEL, 2, s)
        except Stall:
            old = None
        if old is None:
            _expect_stall(chk, lambda: dev.pget("CS.AUX_STATE", addr=s), "VC_ERR_STATE", f"AUX_STATE {s}, not an aux")
            _expect_stall(chk, lambda: dev.pget("CS.AUX_LEVEL", addr=s), "VC_ERR_STATE", f"AUX_LEVEL {s}, not an aux")
        else:
            chk.eq((dev.pget("CS.AUX_STATE", addr=s), dev.pget("CS.AUX_LEVEL", addr=s)), old, f"AUX {s}")
    _same(chk, lambda: dev.pget("CS.AUX_ALL"), lambda: dev.raw_get(OP.GET_CS_AUX_STATE, 48, 0xFFFF),
          "AUX_ALL == 0x05 @0xFFFF")
    for i in range(lim["ir"]):
        _same(chk, lambda: dev.pget("CS.IR_CMD", addr=i), lambda: dev.raw_get(OP.GET_CS_IR_CMD, 16, i), f"IR_CMD {i}")
    for g in range(lim["groups"]):
        _same(chk, lambda: dev.pget("CS.GROUP", addr=g), lambda: dev.raw_get(OP.GET_CS_GROUP, 40, g), f"GROUP {g}")
    for m in range(lim["macros"]):
        old = dev.raw_get(OP.GET_CS_MACRO, 132, m)
        chk.eq(dev.pget("CS.MACRO", addr=m), old, f"MACRO {m}")
        chk.eq(dev.pget("CS.MACRO_HEADER", addr=m), old[:33] + bytes(3), f"MACRO_HEADER {m} == header, reserved 0")
        for k in range(lim["steps"]):
            chk.eq(dev.pget("CS.MACRO_STEP", addr=(k << 8) | m), old[36 + 12 * k:48 + 12 * k],
                   f"MACRO_STEP {k} of macro {m}")
    old = dev.raw_get(OP.GET_CS_DISPLAY_CFG, 16)
    chk.eq(dev.pget("CS.DISPLAY_CAPS"), old[:4], "DISPLAY_CAPS == 0x28 bytes 0-3")
    chk.eq(dev.pget("CS.DISPLAY_CONFIG"), old[4:], "DISPLAY_CONFIG == 0x28 bytes 4-15")
    for p in range(lim["pages"]):
        _same(chk, lambda: dev.pget("CS.DISPLAY_PAGE", addr=p), lambda: dev.raw_get(OP.GET_CS_DISPLAY_PAGE, 4, p),
              f"DISPLAY_PAGE {p}")
    _same(chk, lambda: dev.pget("CS.DISPLAY_STATUS"), lambda: dev.raw_get(OP.GET_CS_DISPLAY_STATUS, 8),
          "DISPLAY_STATUS == 0x2B")


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
    """IR_LEARN_CANCEL while idle, MACRO_FIRE of an empty macro and MACRO_CANCEL succeed as no-ops, old and new."""
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
    chk.eq(dev.pset("CS.IR_LEARN_CANCEL"), 0, "IR_LEARN_CANCEL accepted")
    chk.eq(dev.raw_get(OP.CS_IR_LEARN, 1, 0), b"\x01", "old 0x8F @0 acknowledges")
    chk.eq(dev.pget("CS.IR_LEARN_STATUS")[0], 0, "learn still idle")
    try:
        r = dev.raw_get(OP.CS_IR_LEARN, 8, 3)
        chk.ok(False, f"old 0x8F @3 should STALL, got {r!r}")
    except Stall:
        chk.ok(True, "old 0x8F @3 STALLs")
    m = empty[-1]
    chk.eq(dev.pset("CS.MACRO_FIRE", addr=m), 0, f"MACRO_FIRE of empty macro {m}")
    chk.eq(dev.pget("CS.EXT_STATUS")[3], 0xFF, "an empty macro never runs")
    chk.eq(dev.pset("CS.MACRO_CANCEL"), 0, "MACRO_CANCEL while idle")
    chk.eq(dev.raw_get(OP.CS_MACRO_FIRE, 1, m), b"\x01", f"old 0x25 @{m} acknowledges")
    chk.eq(dev.raw_get(OP.CS_MACRO_FIRE, 1, 0xFFFF), b"\x01", "old 0x25 @0xFFFF acknowledges")
    chk.eq(dev.pget("CS.EXT_STATUS"), ext, "EXT_STATUS unchanged")
    chk.eq(_status(dev)["raw"], before, "CS.STATUS unchanged")


@test("paged_cs", mutating=True)
def pcs_rejections_report_status(dev, profile, chk):
    """AUX SETs on a non-aux slot and IR_LEARN_ARM without a receiver STALL and post the reason, old and new.

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
    # 1.1.6 ACKs every OUT on USB, so the old opcode reports the refusal through the status only.
    s2 = free[-2] if len(free) > 1 else s   # another empty slot, so the post is visibly the old opcode's
    dev.raw_set(OP.SET_CS_AUX_STATE, b"\x00", s2)
    st = _status(dev)
    chk.eq((st["last"], st["slot"]), (INVALID_AUX, s2), "old SET_CS_AUX_STATE posts INVALID_AUX")
    if TYPE_IR in types:
        chk.note("an IR receiver binding exists: IR_LEARN_ARM not sent")
        return
    _expect_stall(chk, lambda: dev.pset("CS.IR_LEARN_ARM"), "VC_ERR_STATE", "IR_LEARN_ARM without a receiver")
    st = _status(dev)
    chk.eq((st["last"], st["slot"]), (NO_IR, 0xFF), "IR_LEARN_ARM posts NO_IR")
    chk.eq(dev.pget("CS.IR_LEARN_STATUS")[0], 0, "learn still idle")


@test("paged_cs", mutating=True, flash=1)
def pcs_preview_roundtrip_and_save(dev, profile, chk):
    """Preview SETs on empty records round-trip old and new, are restored, then CS.SAVE of identical content clears dirty."""
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

    def put(name, payload, addr, tag, old_op=None):
        if old_op is None:
            dev.pset(name, payload, addr=addr)
        else:
            dev.raw_set(old_op, payload, addr)
        st = _settle(dev)
        chk.ok(st is not None and (st["last"], st["slot"]) == (0, tag),
               f"{name} @0x{addr:04X} applied: {st and (hex(st['last']), hex(st['slot']))}")

    try:
        rec = bytes([0, 17, 0, 0, 0, 0, 0, 1]) + bytes(16)   # CS_TYPE_NONE: stored, never claims a pin
        put("CS.BINDING", rec, slot, slot)
        chk.eq(dev.pget("CS.BINDING", addr=slot), rec, "BINDING round trip")
        chk.eq(dev.raw_get(OP.GET_CS_BINDING, 24, slot), rec, "BINDING visible through 0x85")
        rec2 = bytes([0, 18]) + rec[2:]
        put("CS.BINDING", rec2, slot, slot, OP.SET_CS_BINDING)
        chk.eq(dev.pget("CS.BINDING", addr=slot), rec2, "old 0x84 write visible through BINDING")

        put("CS.SLOT_NAME", b"PagedName", slot, slot)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"PagedName".ljust(32, b"\0"), "SLOT_NAME round trip")
        chk.eq(dev.raw_get(OP.GET_CS_NAME, 32, slot), b"PagedName".ljust(32, b"\0"), "SLOT_NAME through 0x8C")
        put("CS.SLOT_NAME", b"A" * 32, slot, slot)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"A" * 31 + b"\0", "32-byte name keeps a NUL terminator")
        put("CS.SLOT_NAME", b"OldName", slot, slot, OP.SET_CS_NAME)
        chk.eq(dev.pget("CS.SLOT_NAME", addr=slot), b"OldName".ljust(32, b"\0"), "old 0x8B write visible")

        put("CS.IR_CMD", IR_REC, sub, 0x80 | sub)
        chk.eq(dev.pget("CS.IR_CMD", addr=sub), IR_REC, "IR_CMD round trip")
        chk.eq(dev.raw_get(OP.GET_CS_IR_CMD, 16, sub), IR_REC, "IR_CMD through 0x8E")

        put("CS.GROUP", GROUP_REC, grp, 0x40 | grp)
        chk.eq(dev.pget("CS.GROUP", addr=grp), GROUP_REC, "GROUP round trip")
        chk.eq(dev.raw_get(OP.GET_CS_GROUP, 40, grp), GROUP_REC, "GROUP through 0x21")
        old_grp = GROUP_REC[:8] + b"OldGrp".ljust(32, b"\0")
        put("CS.GROUP", old_grp, grp, 0x40 | grp, OP.SET_CS_GROUP)
        chk.eq(dev.pget("CS.GROUP", addr=grp), old_grp, "old 0x20 write visible through GROUP")

        put("CS.MACRO_STEP", STEP0, mac, 0x60 | mac)
        chk.eq(dev.pget("CS.MACRO_STEP", addr=mac), STEP0, "MACRO_STEP round trip")
        put("CS.MACRO_STEP", STEP1, (1 << 8) | mac, 0x60 | mac, OP.SET_CS_MACRO_STEP)
        chk.eq(dev.pget("CS.MACRO_STEP", addr=(1 << 8) | mac), STEP1, "old 0x24 write visible through MACRO_STEP")
        hdr = struct.pack("<32sBBBB", b"PagedMacro", 0, 0, 0, 0)
        put("CS.MACRO_HEADER", hdr, mac, 0x60 | mac)
        chk.eq(dev.pget("CS.MACRO_HEADER", addr=mac), hdr, "MACRO_HEADER round trip")
        m = dev.pget("CS.MACRO", addr=mac)
        chk.eq((m[:36], m[36:48], m[48:60]), (hdr, STEP0, STEP1), "MACRO carries header and steps")
        chk.eq(dev.raw_get(OP.GET_CS_MACRO, 132, mac), m, "MACRO through 0x23")

        put("CS.DISPLAY_PAGE", PAGE_REC, page, 0x50 | page)
        chk.eq(dev.pget("CS.DISPLAY_PAGE", addr=page), PAGE_REC, "DISPLAY_PAGE round trip")
        chk.eq(dev.raw_get(OP.GET_CS_DISPLAY_PAGE, 4, page), PAGE_REC, "DISPLAY_PAGE through 0x2A")

        cfg = bytearray(snap["display"])
        cfg[6] ^= 1                                   # brightness, one step
        put("CS.DISPLAY_CONFIG", bytes(cfg), 0, 0x50)
        chk.eq(dev.pget("CS.DISPLAY_CONFIG"), bytes(cfg), "DISPLAY_CONFIG round trip")
        chk.eq(dev.raw_get(OP.GET_CS_DISPLAY_CFG, 16)[4:], bytes(cfg), "DISPLAY_CONFIG through 0x28")
        put("CS.DISPLAY_CONFIG", snap["display"], 0, 0x50, OP.SET_CS_DISPLAY_CFG)
        chk.eq(dev.pget("CS.DISPLAY_CONFIG"), snap["display"], "old 0x27 write visible")
    finally:
        put("CS.BINDING", snap["binding"][slot], slot, slot)
        put("CS.SLOT_NAME", _name_payload(snap["name"][slot]), slot, slot)
        put("CS.IR_CMD", snap["ir"][sub], sub, 0x80 | sub)
        put("CS.GROUP", snap["group"][grp], grp, 0x40 | grp)
        put("CS.MACRO_HEADER", snap["macro"][mac][:36], mac, 0x60 | mac)
        for k in range(lim["steps"]):
            step = snap["macro"][mac][36 + 12 * k:48 + 12 * k]
            if dev.pget("CS.MACRO_STEP", addr=(k << 8) | mac) != step:
                put("CS.MACRO_STEP", step, (k << 8) | mac, 0x60 | mac)
        put("CS.DISPLAY_PAGE", snap["page"][page], page, 0x50 | page)
        put("CS.DISPLAY_CONFIG", snap["display"], 0, 0x50)

    after = _snapshot(dev, lim)
    diff = [k for k in snap if snap[k] != after[k]]
    if diff:
        chk.ok(False, f"records differ from the snapshot after restore ({diff}); CS.SAVE NOT sent, preview left dirty")
        return
    chk.ok(True, "every CS record equals the pre-test snapshot")
    chk.eq(_status(dev)["dirty"], 1, "restored preview is still marked dirty")
    chk.eq(dev.pset("CS.SAVE"), 0, "CS.SAVE accepted")
    time.sleep(0.3)
    dev.wait_ready()
    st = _settle(dev, timeout_s=5.0)
    chk.ok(st is not None and (st["last"], st["slot"], st["dirty"]) == (0, 0xFF, 0),
           f"SAVE completes clean: {st and (hex(st['last']), hex(st['slot']), st['dirty'])}")
    chk.eq(_snapshot(dev, lim), snap, "records unchanged by the SAVE")
