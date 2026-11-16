"""
Native paged tests for the SYS page (plan sections 5.1 and 6.1): discovery, identity, device name and
telemetry. Every test SKIPs on firmware without the paged dispatcher or the command it covers.
ENTER_BOOTLOADER is never sent, in either direction. The device name test writes flash.
Registry comparisons use Documentation/protocol/dspi_commands.json.
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


# --- Device name and telemetry (plan 5.1, 6.1) ---------------------------------

# Old GET_STATUS selector -> (SYS command, struct format, offset, how the reads compare).
# "eq" must match, "up" only grows between reads, "down" only shrinks, "temp" is within 3 C.
STATUS_FIELDS = {
    3: ("SYS.PDM_STATS", "<I", 0, "up"), 4: ("SYS.PDM_STATS", "<I", 4, "up"),
    5: ("SYS.PDM_STATS", "<I", 8, "up"), 6: ("SYS.PDM_STATS", "<I", 12, "up"),
    8: ("SYS.SPDIF_TX_STATS", "<I", 0, "up"),
    10: ("SYS.USB_AUDIO_STATUS", "<I", 0, "up"), 11: ("SYS.USB_AUDIO_STATUS", "<B", 12, "eq"),
    12: ("SYS.USB_AUDIO_STATUS", "<B", 13, "eq"),
    13: ("SYS.SYSTEM_STATUS", "<I", 0, "eq"), 14: ("SYS.SYSTEM_STATUS", "<H", 8, "eq"),
    15: ("SYS.SYSTEM_STATUS", "<I", 4, "eq"), 16: ("SYS.SYSTEM_STATUS", "<h", 10, "temp"),
    17: ("SYS.SPDIF_TX_STATS", "<I", 4, "up"), 18: ("SYS.SPDIF_TX_STATS", "<I", 8, "up"),
    19: ("SYS.SPDIF_TX_STATS", "<I", 12, "up"), 20: ("SYS.SPDIF_TX_STATS", "<I", 16, "up"),
    21: ("SYS.SPDIF_TX_STATS", "<I", 20, "up"), 22: ("SYS.USB_AUDIO_STATUS", "<I", 4, "up"),
    26: ("SYS.PIPELINE_STATS", "<I", 0, "up"), 27: ("SYS.PIPELINE_STATS", "<I", 4, "up"),
    28: ("SYS.PIPELINE_STATS", "<I", 8, "down"), 29: ("SYS.PIPELINE_STATS", "<I", 12, "up"),
    30: ("SYS.USB_AUDIO_STATUS", "<I", 8, "up"), 31: ("SYS.PIPELINE_STATS", "<I", 16, "up"),
}
TELEMETRY = {"SYS.SYSTEM_STATUS": (16, (14, 16)), "SYS.USB_AUDIO_STATUS": (16, (14, 16)),
             "SYS.BUFFER_STATS": (44, None), "SYS.PIPELINE_STATS": (24, (20, 24)),
             "SYS.PDM_STATS": (16, None), "SYS.SPDIF_TX_STATS": (24, None)}
_WIDTH_MASK = {"<B": 0xFF, "<H": 0xFFFF}


def _has(dev, name):
    """True when the attached firmware implements the GET or SET of `name`."""
    info = dev.resolve(name)
    gets, sets = dev.paged_commands().get(info.page, ((), ()))
    return info.cmd in gets or info.cmd in sets


def _need(dev, *names):
    _paged(dev)
    missing = [n for n in names if not _has(dev, n)]
    if missing:
        raise Skip(f"firmware lacks {', '.join(missing)}")


class _NoLegacy(Exception):
    """Page 0 no longer answers the 1.1.6 opcode, so there is nothing to compare against."""


def _old(fn):
    try:
        return fn()
    except Stall as e:
        raise _NoLegacy(str(e)) from None


def _legacy_status(dev, sel, fmt):
    raw = _old(lambda: dev.raw_get(OP.GET_STATUS, 4, sel))
    v = struct.unpack("<i" if fmt in ("<h", "<i") else "<I", raw)[0]
    return v & _WIDTH_MASK[fmt] if fmt in _WIDTH_MASK else v


def _bracketed(kind, before, new, after):
    if kind == "eq":
        return before == new == after
    if kind == "up":
        return before <= new <= after
    if kind == "down":
        return before >= new >= after
    return abs(new - before) <= 300 and abs(new - after) <= 300


@test("paged_sys")
def telemetry_matches_get_status(dev, profile, chk):
    """Every GET_STATUS selector that moved to a SYS telemetry struct reads the same source."""
    _need(dev, *sorted({f[0] for f in STATUS_FIELDS.values()}))
    try:
        for sel, (name, fmt, off, kind) in STATUS_FIELDS.items():
            for _ in range(3):
                before = _legacy_status(dev, sel, fmt)
                new = struct.unpack_from(fmt, dev.pget(name), off)[0]
                after = _legacy_status(dev, sel, fmt)
                if _bracketed(kind, before, new, after):
                    break
            chk.ok(_bracketed(kind, before, new, after),
                   f"GET_STATUS {sel} vs {name}+{off}: old {before}, new {new}, old again {after} ({kind})")
    except _NoLegacy:
        chk.note("page 0 no longer answers GET_STATUS: comparison skipped")


@test("paged_sys")
def telemetry_cpu_loads(dev, profile, chk):
    """SYS.SYSTEM_STATUS CPU bytes are loads, near the ones GET_STATUS 2 packs beside peak 4."""
    _need(dev, "SYS.SYSTEM_STATUS")
    new = dev.pget("SYS.SYSTEM_STATUS")
    chk.in_range(new[12], 0, 100, "cpu0 load")
    chk.in_range(new[13], 0, 100, "cpu1 load")
    try:
        old = _old(lambda: dev.raw_get(OP.GET_STATUS, 4, 2))
    except _NoLegacy:
        chk.note("page 0 no longer answers GET_STATUS: comparison skipped")
        return
    chk.ok(abs(new[12] - old[2]) <= 20 and abs(new[13] - old[3]) <= 20,
           f"loads near GET_STATUS 2: new {new[12]}/{new[13]}, old {old[2]}/{old[3]}")


@test("paged_sys")
def telemetry_shapes_and_errors(dev, profile, chk):
    """Telemetry GETs have their table lengths and zero reserved bytes; SETs and wValue 1 are refused."""
    _paged(dev)
    present = [n for n in TELEMETRY if _has(dev, n)]
    if not present:
        raise Skip("firmware has no SYS telemetry command")
    for name in present:
        n, reserved = TELEMETRY[name]
        raw = dev.pget(name, length=n)
        chk.eq(len(raw), n, f"{name} length")
        if reserved:
            chk.eq(raw[reserved[0]:reserved[1]], bytes(reserved[1] - reserved[0]), f"{name} reserved bytes")
        _expect_stall(chk, lambda: dev.pset(name, b"\x00"), "VC_ERR_DIRECTION", f"SET {name}")
        _expect_stall(chk, lambda: dev.pget(name, addr=1, length=n), "VC_ERR_ADDRESS", f"{name} at wValue 1")


@test("paged_sys")
def buffer_stats_matches_legacy(dev, profile, chk):
    """SYS.BUFFER_STATS is the 0xB0 packet: same header, one sequence counter shared with 0xB0."""
    _need(dev, "SYS.BUFFER_STATS")
    seq = lambda b: struct.unpack_from("<H", b, 2)[0]  # noqa: E731
    new = dev.pget("SYS.BUFFER_STATS", length=44)
    chk.eq(len(new), 44, "length")
    if profile is not None:
        chk.eq(new[0], profile.num_spdif, "num_spdif matches the profile")
    for i in range(4):
        base = 4 + i * 8
        free, prep, playing, fill, mn, mx = new[base:base + 6]
        chk.in_range(free, 0, 16, f"spdif[{i}] consumer_free")
        chk.member(playing, (0, 1), f"spdif[{i}] consumer_playing")
        chk.in_range(fill, 0, 100, f"spdif[{i}] fill_pct")
        if i >= new[0]:
            chk.eq(new[base:base + 8], bytes(8), f"spdif[{i}] unused instance zero")
    if not new[1] & 0x01:
        chk.eq(new[36:44], bytes(8), "PDM block zero while PDM is off")
    try:
        old = _old(lambda: dev.raw_get(OP.GET_BUFFER_STATS, 44))
        new = dev.pget("SYS.BUFFER_STATS", length=44)
        old2 = _old(lambda: dev.raw_get(OP.GET_BUFFER_STATS, 44))
    except _NoLegacy:
        chk.note("page 0 no longer answers GET_BUFFER_STATS: comparison skipped")
        return
    chk.eq(new[0], old[0], "num_spdif == 0xB0")
    chk.eq(new[1] & 0x01, old[1] & 0x01, "PDM active flag == 0xB0")
    chk.ok(0 < (seq(new) - seq(old)) & 0xFFFF <= 3 and 0 < (seq(old2) - seq(new)) & 0xFFFF <= 3,
           f"one sequence counter: 0xB0 {seq(old)}, SYS {seq(new)}, 0xB0 {seq(old2)}")


@test("paged_sys", mutating=True)
def stats_reset(dev, profile, chk):
    """SYS.STATS_RESET bit 0 reseeds the watermarks; mask 0 is a no-op; undefined bits and bad sizes are refused."""
    _need(dev, "SYS.STATS_RESET")
    _expect_stall(chk, lambda: dev.pset("SYS.STATS_RESET", b"\x02"), "VC_ERR_VALUE", "undefined bit 1")
    _expect_stall(chk, lambda: dev.pset("SYS.STATS_RESET", b"\x81"), "VC_ERR_VALUE", "bit 0 with bit 7")
    _expect_stall(chk, lambda: dev.pset("SYS.STATS_RESET"), "VC_ERR_LENGTH", "zero-length SET")
    _expect_stall(chk, lambda: dev.pset("SYS.STATS_RESET", b"\x01\x00"), "VC_ERR_LENGTH", "2-byte SET")
    _expect_stall(chk, lambda: dev.pset("SYS.STATS_RESET", b"\x01", addr=1), "VC_ERR_ADDRESS", "wValue 1")
    _expect_stall(chk, lambda: dev.pget("SYS.STATS_RESET", length=1), "VC_ERR_DIRECTION", "GET")
    chk.no_stall(lambda: dev.pset("SYS.STATS_RESET", b"\x00"), "mask 0 accepted")
    chk.no_stall(lambda: dev.pset("SYS.STATS_RESET", b"\x01"), "bit 0 accepted")
    if _has(dev, "SYS.BUFFER_STATS"):
        data = dev.pget("SYS.BUFFER_STATS", length=44)
        for i in range(data[0]):
            mn, mx = data[4 + i * 8 + 4], data[4 + i * 8 + 5]
            chk.ok(mn <= mx or (mn, mx) == (100, 0), f"spdif[{i}] watermarks coherent after reset ({mn}/{mx})")
    try:
        chk.eq(_old(lambda: dev.raw_get(OP.RESET_BUFFER_STATS, 1, 1)), b"\x01", "0xB1 still acks")
    except _NoLegacy:
        chk.note("page 0 no longer answers RESET_BUFFER_STATS")


@test("paged_sys", mutating=True, flash=6)
def device_name_roundtrip(dev, profile, chk):
    """SYS.DEVICE_NAME round-trips, agrees with 0x56/0x57 both ways, keeps 31 bytes, refuses bad sizes and wValue."""
    _need(dev, "SYS.DEVICE_NAME")
    name = lambda: dev.pget("SYS.DEVICE_NAME", length=32)  # noqa: E731
    orig = name()
    chk.eq(len(orig), 32, "GET is 32 bytes")
    try:
        dev.pset("SYS.DEVICE_NAME", b"QA Paged")
        got = name()
        chk.eq(got.split(b"\x00", 1)[0], b"QA Paged", "paged SET, paged GET")
        dev.pset("SYS.DEVICE_NAME", got)
        chk.eq(name(), got, "a GET reply is a valid SET payload")
        dev.pset("SYS.DEVICE_NAME", b"C" * 32)
        chk.eq(name(), b"C" * 31 + b"\x00", "32-byte SET keeps 31 + NUL")
        _expect_stall(chk, lambda: dev.pset("SYS.DEVICE_NAME", b"D" * 33), "VC_ERR_LENGTH", "33-byte SET")
        _expect_stall(chk, lambda: dev.pset("SYS.DEVICE_NAME"), "VC_ERR_LENGTH", "zero-length SET")
        _expect_stall(chk, lambda: dev.pset("SYS.DEVICE_NAME", b"x", addr=1), "VC_ERR_ADDRESS", "SET wValue 1")
        _expect_stall(chk, lambda: dev.pget("SYS.DEVICE_NAME", addr=1, length=32), "VC_ERR_ADDRESS",
                      "GET wValue 1")
        chk.eq(name(), b"C" * 31 + b"\x00", "refused SETs left the name alone")
        try:
            chk.eq(_old(lambda: dev.raw_get(OP.GET_DEVICE_NAME, 32)), name(), "paged SET, 0x57 GET")
            _old(lambda: dev.raw_set(OP.SET_DEVICE_NAME, b"QA Legacy"))
            chk.eq(name().split(b"\x00", 1)[0], b"QA Legacy", "0x56 SET, paged GET")
        except _NoLegacy:
            chk.note("page 0 no longer answers 0x56/0x57: comparison skipped")
    finally:
        dev.pset("SYS.DEVICE_NAME", orig.split(b"\x00", 1)[0] or b"\x00")
        dev.wait_ready()
    chk.eq(name(), orig, "name restored")
