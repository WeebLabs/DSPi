"""
Native paged tests for the SYS discovery and identity commands (contract and plan section 5.1).

Every test SKIPs on firmware without the paged dispatcher. ENTER_BOOTLOADER is never sent,
in either direction. Registry comparisons use Documentation/protocol/dspi_commands.json.
"""

import json
import struct

from ..device import (OP, PagedStall, ProtocolError, REGISTRY_REL, Stall, VC_CODES, _find_repo_file)
from ..framework import Skip, test

SYS = 0x01
UNUSED_PAGE = 0xDF          # inside the reserved 0x80-0xDF block, never allocated
NEVER_SEND = {"ENTER_BOOTLOADER", "FACTORY_RESET", "PRESET_SAVE", "PRESET_LOAD", "PRESET_DELETE"}
LEGACY_TWINS = [("PLATFORM", "GET_PLATFORM", 7), ("SERIAL", "GET_SERIAL", 16),
                ("BUILD_INFO", "GET_BUILD_INFO", 64), ("CORE1_MODE", "GET_CORE1_MODE", 1)]


def _paged(dev):
    try:
        info = dev.protocol_info()
    except ProtocolError as e:
        raise AssertionError(str(e)) from None
    if info is None:
        raise Skip("no paged dispatcher: SYS.PROTOCOL_INFO STALLs (firmware 1.1.6 or older)")
    return info


def _registry(dev):
    if isinstance(getattr(dev, "registry", None), dict):
        return dev.registry
    p = _find_repo_file(REGISTRY_REL)
    if p is None:
        raise Skip(f"{REGISTRY_REL} not generated yet")
    return json.loads(p.read_text())


def _sys_page(reg):
    for pg in reg["pages"]:
        if pg["page"] == SYS:
            return pg
    raise Skip("registry has no SYS page")


def _expect_stall(chk, fn, code, msg, retries=3):
    """fn() must STALL with VC code `code`. Retried because LAST_RESULT is shared by all USB hosts."""
    seen = None
    for _ in range(retries):
        try:
            r = fn()
        except PagedStall as s:
            if s.result is not None:
                return chk.eq(s.result_name, code, f"{msg}: LAST_RESULT")
            seen = s
            continue
        return chk.ok(False, f"{msg}: expected STALL with {code}, got {r!r}")
    return chk.ok(False, f"{msg}: STALLed but LAST_RESULT unreadable or overwritten ({seen})")


def _bits(bitmap: bytes) -> set:
    return {i for i in range(len(bitmap) * 8) if bitmap[i // 8] >> (i % 8) & 1}


@test("paged_sys")
def protocol_info_fields(dev, profile, chk):
    """SYS.PROTOCOL_INFO: "DSPi" magic, protocol version, page count, transfer limits, reserved zero."""
    info = _paged(dev)
    raw = dev.pget(0x0110, length=16)
    chk.eq(len(raw), 16, "reply length")
    chk.eq(raw[:4], b"DSPi", "magic")
    chk.eq(info["flags"], 0, "flags")
    chk.eq(info["max_set"], 64, "largest non-bulk SET payload")
    chk.eq(info["max_get"], 256, "largest non-bulk GET reply")
    chk.eq(info["reserved"], 0, "reserved")
    chk.eq(info["page_count"], len(dev.pget(0x0111, length=256)) // 2, "page count matches PAGE_LIST")
    try:
        reg = _registry(dev)
    except Skip as s:
        chk.note(str(s))
        return
    chk.eq((info["major"], info["minor"]), (reg["protocol"]["major"], reg["protocol"]["minor"]),
           "protocol version matches the registry")


@test("paged_sys")
def page_list_ascending(dev, profile, chk):
    """SYS.PAGE_LIST: {page, revision} pairs, strictly ascending, SYS present, page 0 absent."""
    _paged(dev)
    raw = dev.pget(0x0111, length=256)
    chk.eq(len(raw) % 2, 0, "even length")
    pairs = [(raw[i], raw[i + 1]) for i in range(0, len(raw) - 1, 2)]
    pages = [p for p, _ in pairs]
    chk.ok(SYS in pages, f"SYS (0x01) listed: {pages}")
    chk.ok(0 not in pages, "page 0 never listed")
    chk.ok(all(a < b for a, b in zip(pages, pages[1:])), f"ascending: {pages}")
    for p in pages:
        bm = dev.pget(0x0112, addr=p, length=64)
        chk.ok(any(bm), f"page 0x{p:02X} listed but PAGE_COMMANDS is empty")
    try:
        sysrev = _sys_page(_registry(dev))["revision"]
    except Skip as s:
        chk.note(str(s))
        return
    chk.eq(dict(pairs).get(SYS), sysrev, "SYS revision matches the registry")


@test("paged_sys")
def page_commands_sys_matches_registry(dev, profile, chk):
    """SYS.PAGE_COMMANDS for SYS equals the registry's GET and SET sets; an unlisted page is VC_ERR_ADDRESS."""
    _paged(dev)
    sys_pg = _sys_page(_registry(dev))
    bm = dev.pget(0x0112, addr=SYS, length=64)
    chk.eq(len(bm), 64, "bitmap length")
    got_get, got_set = _bits(bm[:32]), _bits(bm[32:64])
    for c in sys_pg["commands"]:
        if c.get("build"):
            continue   # present only in some builds
        chk.eq(c["cmd"] in got_get, bool(c["get"]), f"GET bit of SYS.{c['name']}")
        chk.eq(c["cmd"] in got_set, bool(c["set"]), f"SET bit of SYS.{c['name']}")
    known = {c["cmd"] for c in sys_pg["commands"]}
    chk.eq(sorted((got_get | got_set) - known), [], "bits set for commands the registry lacks")
    chk.eq(sorted(n for n in got_get | got_set if n < 0x10), [], "no bit below 0x10")
    _expect_stall(chk, lambda: dev.pget(0x0112, addr=UNUSED_PAGE, length=64), "VC_ERR_ADDRESS",
                  "PAGE_COMMANDS of an unimplemented page")


@test("paged_sys")
def command_info_sys_matches_registry(dev, profile, chk):
    """SYS.COMMAND_INFO for each SYS command matches the registry sizes and directions."""
    _paged(dev)
    sys_pg = _sys_page(_registry(dev))
    addr_byte: dict[str, int] = {}
    for c in sys_pg["commands"]:
        r = dev.pget(0x0113, addr=(SYS << 8) | c["cmd"], length=8)
        if not chk.eq(len(r), 8, f"SYS.{c['name']} reply length"):
            continue
        flags, get_len, smin, smax, akind, has = struct.unpack("<HHBBBB", r)
        want_len = 0xFFFF if c["get_len"] == "var" else (c["get_len"] or 0)
        chk.eq(get_len, want_len if c["get"] else 0, f"SYS.{c['name']} get_len")
        chk.eq((smin, smax), (c["set_min"], c["set_max"]), f"SYS.{c['name']} SET bounds")
        chk.eq(has & 3, (1 if c["get"] else 0) | (2 if c["set"] else 0), f"SYS.{c['name']} has GET/SET bits")
        if not c["flags"]:
            chk.eq(flags, 0, f"SYS.{c['name']} flags")
        prev = addr_byte.setdefault(c["addr"], akind)
        chk.eq(akind, prev, f"SYS.{c['name']} addr kind byte consistent for {c['addr']}")
    chk.eq(len(set(addr_byte.values())), len(addr_byte), f"distinct addr kinds distinct bytes: {addr_byte}")


@test("paged_sys")
def command_info_missing_command(dev, profile, chk):
    """SYS.COMMAND_INFO of a command that does not exist STALLs with VC_ERR_ADDRESS."""
    _paged(dev)
    sys_pg = _sys_page(_registry(dev))
    used = {c["cmd"] for c in sys_pg["commands"]} | {r["cmd"] for r in sys_pg.get("reserved", [])}
    free = next(n for n in range(0xFF, 0x0F, -1) if n not in used)
    _expect_stall(chk, lambda: dev.pget(0x0113, addr=(SYS << 8) | free, length=8), "VC_ERR_ADDRESS",
                  f"COMMAND_INFO of SYS 0x{free:02X}")
    _expect_stall(chk, lambda: dev.pget(0x0113, addr=(UNUSED_PAGE << 8) | 0x10, length=8), "VC_ERR_ADDRESS",
                  "COMMAND_INFO on an unimplemented page")


@test("paged_sys")
def identity_matches_legacy(dev, profile, chk):
    """SYS.PLATFORM / SERIAL / BUILD_INFO / CORE1_MODE are byte-identical to their 1.1.6 opcodes."""
    _paged(dev)
    for new, old, n in LEGACY_TWINS:
        paged = dev.pget(f"SYS.{new}", length=n)
        chk.eq(len(paged), n, f"SYS.{new} length")
        try:
            legacy = dev.raw_get(getattr(OP, old), n)
        except Stall:
            chk.note(f"page 0 no longer answers {old}: comparison skipped")
            continue
        chk.eq(paged, legacy, f"SYS.{new} == {old}")


@test("paged_sys")
def unknown_page_and_command(dev, profile, chk):
    """An unknown page or command STALLs both ways and LAST_RESULT reads VC_ERR_UNKNOWN."""
    _paged(dev)
    sys_cmds = {c["cmd"] for c in _sys_page(_registry(dev))["commands"]}
    free = next(n for n in range(0xFE, 0x0F, -1) if n not in sys_cmds)
    for cid, what in (((UNUSED_PAGE << 8) | 0x10, "unknown page"), ((SYS << 8) | free, "unknown SYS command")):
        _expect_stall(chk, lambda: dev.pget(cid, length=8), "VC_ERR_UNKNOWN", f"GET {what}")
        _expect_stall(chk, lambda: dev.pset(cid, b"\x00"), "VC_ERR_UNKNOWN", f"SET {what}")


@test("paged_sys")
def wrong_direction(dev, profile, chk):
    """A SET to a GET-only command (and a GET to a safe SET-only one) gives VC_ERR_DIRECTION."""
    _paged(dev)
    _expect_stall(chk, lambda: dev.pset("SYS.PLATFORM"), "VC_ERR_DIRECTION", "zero-length SET to SYS.PLATFORM")
    _expect_stall(chk, lambda: dev.pset("SYS.PLATFORM", b"\x01"), "VC_ERR_DIRECTION", "SET with payload")
    for c in _sys_page(_registry(dev))["commands"]:
        if c["set"] and not c["get"] and c["name"] not in NEVER_SEND and not c["flags"]:
            _expect_stall(chk, lambda: dev.pget((SYS << 8) | c["cmd"], length=8), "VC_ERR_DIRECTION",
                          f"GET to SET-only SYS.{c['name']}")
            break
    else:
        chk.note("no SET-only SYS command safe to probe with a GET yet")


@test("paged_sys")
def set_bad_length(dev, profile, chk):
    """A SET whose payload is outside the command's bounds gives VC_ERR_LENGTH."""
    _paged(dev)
    reg = _registry(dev)
    cand = [(pg["page"], c) for pg in reg["pages"] for c in pg["commands"]
            if c["set"] and c["name"] not in NEVER_SEND and "BULK" not in c["flags"]
            and not {"FLASH", "RESTART"} & set(c["flags"]) and c["set_max"] < 64]
    if not cand:
        raise Skip("no SET command besides ENTER_BOOTLOADER in the registry yet")
    page, c = min(cand, key=lambda pc: (pc[0] != SYS, pc[0], pc[1]["cmd"]))
    cid = (page << 8) | c["cmd"]
    _expect_stall(chk, lambda: dev.pset(cid, bytes(c["set_max"] + 1)), "VC_ERR_LENGTH",
                  f"{c['set_max'] + 1}-byte SET to {c['name']}")
    if c["set_min"] > 1:
        _expect_stall(chk, lambda: dev.pset(cid, bytes(c["set_min"] - 1)), "VC_ERR_LENGTH",
                      f"{c['set_min'] - 1}-byte SET to {c['name']}")


@test("paged_sys")
def none_address_rejects_wvalue(dev, profile, chk):
    """A command whose address kind is NONE refuses a non-zero wValue with VC_ERR_ADDRESS."""
    _paged(dev)
    for name in ("SYS.PLATFORM", "SYS.PROTOCOL_INFO"):
        _expect_stall(chk, lambda: dev.pget(name, addr=1), "VC_ERR_ADDRESS", f"{name} at wValue 1")


@test("paged_sys")
def get_truncated_to_wlength(dev, profile, chk):
    """GET replies are truncated to wLength, so a short read is a prefix of the full one."""
    _paged(dev)
    for cid, full, short in ((0x0120, 7, 4), (0x0122, 64, 10), (0x0110, 16, 8)):
        a, b = dev.pget(cid, length=full), dev.pget(cid, length=short)
        chk.eq(len(b), short, f"0x{cid:04X} length at wLength {short}")
        chk.eq(b, a[:short], f"0x{cid:04X} prefix at wLength {short}")


@test("paged_sys")
def last_result_tracks_requests(dev, profile, chk):
    """SYS.LAST_RESULT records ID, direction, VC_OK and wValue of the last request, never itself."""
    _paged(dev)
    for _ in range(3):
        dev.pget("SYS.PLATFORM", length=7)
        first, second = dev.last_result(), dev.last_result()
        if first and first.cmd_id == 0x0120:
            break
    chk.ok(first is not None, "LAST_RESULT answers")
    if first is None:
        return
    chk.eq((first.cmd_id, first.direction, first.result, first.wvalue), (0x0120, "GET", VC_CODES["VC_OK"], 0),
           "after GET SYS.PLATFORM")
    chk.eq(second, first, "reading LAST_RESULT does not record itself")
    raw = dev.pget(0x0114, length=8)
    chk.eq(raw[6:8], b"\x00\x00", "reserved bytes")
