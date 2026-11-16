"""
Whole-surface contract checks on the paged protocol, driven by the device's discovery data
(SYS.PAGE_LIST, PAGE_COMMANDS, COMMAND_INFO) and Documentation/protocol/dspi_commands.json.
Every SET sent here is refused by the dispatcher before any handler runs (commands.md 1.3);
assert_refused() proves that from the registry and COMMAND_INFO before each one goes out.
"""

import json
import struct
from pathlib import Path

from ..device import PagedStall
from ..framework import Skip, test
from ..helpers import bits, expect_stall, require_paged
from ._cs import AUX_TYPES

REGISTRY = Path(__file__).resolve().parents[3] / "Documentation" / "protocol" / "dspi_commands.json"

# SYS.COMMAND_INFO flag bits and address kind numbers (commands.md 2.4 and 4.5).
FLAG_BITS = {"USB_ONLY": 0x01, "SET_USB_ONLY": 0x02, "BULK": 0x04, "DEFERRED": 0x08, "FLASH": 0x10,
             "RESTART": 0x20, "KEEPALIVE": 0x40}
ADDR_KINDS = ("NONE", "CHANNEL", "INPUT", "OUTPUT", "OUTPUT_ALL", "SLOT", "PIN_OUTPUT", "PRESET", "CH_BAND",
              "IN_OUT", "SOURCE", "SPDIF_RX", "I2S_PAIR", "ROLE", "CS_SLOT", "CS_SUB", "CS_GROUP", "CS_MACRO",
              "CS_MACRO_STEP", "CS_PAGE", "CS_NOUN", "PAGE", "PAGE_CMD", "OFFSET", "INDEX")
CMD_CHECKED = {"PAGE_CMD", "OFFSET", "INDEX"}     # the dispatcher passes any wValue; the command checks it
MAX_SET = 64

# Commands that must never receive a request the device could accept. This module only sends
# refused requests to anything; stress.py also keeps them out of its valid traffic.
NEVER_VALID = frozenset({
    "SYS.ENTER_BOOTLOADER", "SYS.DEVICE_NAME", "SYS.STATS_RESET",
    "STORE.PRESET_SAVE", "STORE.PRESET_LOAD", "STORE.PRESET_DELETE", "STORE.PRESET_NAME", "STORE.STARTUP",
    "STORE.OUTPUT_CONFIG_MODE", "STORE.OUTPUT_CONFIG_SAVE", "STORE.FACTORY_RESET", "STORE.ALL_PARAMS",
    "STORE.ALL_PARAMS_CHUNK",
    "INPUT.SOURCE", "INPUT.SOURCE_NAME", "INPUT.SPDIF_RX_PIN", "INPUT.SPDIF_RX_ENABLE", "INPUT.I2S_RX_PIN",
    "INPUT.I2S_RX_CHANNELS", "INPUT.I2S_RX_RATE", "INPUT.ADAT_RX_ENABLE", "INPUT.ADAT_RX_PIN",
    "INPUT.ADAT_RX_CLOCK_MODE", "INPUT.LG_SOUND_SYNC_ENABLE",
    "OUTPUT.TYPE", "OUTPUT.PIN", "OUTPUT.LATENCY", "OUTPUT.ADAT_TX_ENABLE", "OUTPUT.ADAT_TX_PIN",
    "OUTPUT.DAC_MUTE_CONFIG", "OUTPUT.DAC_MUTE_TEST",
    "ROUTE.OUTPUT_ENABLE", "ROUTE.MASTER_VOLUME_MODE", "ROUTE.MASTER_VOLUME_SAVE",
    "MEAS.RTA_CONFIG", "MEAS.RTA_START", "MEAS.SIGGEN_CONFIG", "MEAS.SIGGEN_START",
    "CS.SAVE", "CS.REVERT", "CS.BINDING", "CS.IR_CMD", "CS.IR_LEARN_ARM", "CS.MACRO_FIRE", "CS.AUX_STATE",
    "CS.AUX_LEVEL", "CS.DISPLAY_CONFIG", "CS.DISPLAY_PAGE", "CS.GROUP", "CS.MACRO_HEADER", "CS.MACRO_STEP",
    "XPORT.UART_CONFIG", "XPORT.I2C_CONFIG",
    "CLOCK.I2S_BCK_PIN", "CLOCK.I2S_CLOCK_PIN_MODE", "CLOCK.I2S_CLOCK_MODE", "CLOCK.MCK_ENABLE",
    "CLOCK.MCK_PIN", "CLOCK.MCK_MULTIPLIER",
    "DIAG.LOOP_PROF_BG_PACE", "DIAG.LOOP_PROF_RESET",
})

# GET addresses where 0 is not a valid instance, each with the documented reason.
VALID_ADDR = {
    "SYS.PAGE_COMMANDS": 0x01,         # page 0 has no commands: VC_ERR_ADDRESS (2.3)
    "SYS.COMMAND_INFO": 0x0113,        # ID 0 is no command: VC_ERR_ADDRESS (2.4)
    "INPUT.SPDIF_RX_ENABLE": 1,        # addresses optional inputs 1 to 3 only (8.2)
    "DIAG.LOOP_PROF_COMMAND": 0x0120,  # any built command ID
}
AUX_GETS = ("CS.AUX_STATE", "CS.AUX_LEVEL")   # VC_ERR_STATE unless the slot is a live aux output (15.6)


# --- Registry and discovery ---------------------------------------------------------------------

def load_commands():
    if not REGISTRY.is_file():
        raise Skip(f"{REGISTRY} not generated")
    doc = json.loads(REGISTRY.read_text())
    cmds = {}
    for pg in doc["pages"]:
        for c in pg["commands"]:
            cmds[f"{pg['mnemonic']}.{c['name']}"] = dict(c, page=pg["page"], flags=set(c["flags"]))
    return doc, cmds


def _expected(c, profile):
    """True/False when the build condition decides presence; None for a bench-build option."""
    if not c["build"]:
        return True
    if c["build"] == "PICO_RP2350":
        return profile.platform_id == 1
    return None


def _present(dev, c, direction):
    gets, sets = dev.paged_commands().get(c["page"], (frozenset(), frozenset()))
    return c["cmd"] in (gets if direction == "GET" else sets)


def command_info(dev, cid):
    flags, get_len, smin, smax, kind, dirs = struct.unpack("<HHBBBB", dev.pget("SYS.COMMAND_INFO", cid, 8))
    return {"flags": flags, "get_len": get_len, "set_min": smin, "set_max": smax, "kind": kind, "dirs": dirs}


def noun_count(dev):
    return dev.pget("CS.CAPS", length=52)[3]


def addr_ranges(profile, nouns):
    """kind -> (last valid wValue, [first refused wValues]) for the dispatcher-checked kinds (4.5)."""
    nc, ni, no = profile.num_channels, profile.num_input_channels, profile.num_output_channels
    simple = {"CHANNEL": nc, "INPUT": ni, "OUTPUT": no, "OUTPUT_ALL": no, "SLOT": profile.num_spdif,
              "PIN_OUTPUT": profile.num_pin_outputs, "PRESET": 10, "SOURCE": 7, "SPDIF_RX": 4,
              "I2S_PAIR": 4 if profile.platform_id == 1 else 1, "ROLE": 2, "CS_SLOT": 16, "CS_SUB": 16,
              "CS_GROUP": 8, "CS_MACRO": 8, "CS_PAGE": 16, "CS_NOUN": nouns}
    out = {k: (n - 1, [n, 0x0100]) for k, n in simple.items()}
    out["NONE"] = (0, [1, 0xFFFF])
    out["CH_BAND"] = ((nc - 1) << 8, [nc << 8, 0xFF00])
    out["IN_OUT"] = (((ni - 1) << 8) | (no - 1), [ni << 8, no])
    out["CS_MACRO_STEP"] = ((7 << 8) | 7, [8 << 8, 8])
    out["PAGE"] = (0xFF, [0x0100])
    return out


def addr_valid(kind, addr, ranges, is_get=True):
    """Whether the dispatcher passes `addr` for this kind; the command may still refuse it (4.5)."""
    last, _bad = ranges[kind]
    hi, lo = addr >> 8, addr & 0xFF
    if kind == "NONE":
        return addr == 0
    if kind == "OUTPUT_ALL" and not is_get and addr == 0xFF:
        return True
    if kind == "CH_BAND":
        return hi <= last >> 8
    if kind in ("IN_OUT", "CS_MACRO_STEP"):
        return hi <= last >> 8 and lo <= (last & 0xFF)
    return addr <= last


def assert_refused(c, info, length, addr, ranges):
    """Raise unless a SET of `length` bytes at `addr` fails the dispatcher's length or address check
    by both the registry's and the device's own bounds. Called before every SET this module sends."""
    bad_len = "BULK" not in c["flags"] and all(n < lo or n > hi for n, lo, hi in (
        (length, c["set_min"], c["set_max"]), (length, info["set_min"], info["set_max"])))
    bad_addr = (c["addr"] in ranges and ADDR_KINDS[info["kind"]] == c["addr"]
                and not addr_valid(c["addr"], addr, ranges, is_get=False))
    if not (bad_len or bad_addr):
        raise AssertionError(f"refusing to send {c['name']} {length} B at 0x{addr:04X}: it could be accepted")


def _aux_live(dev):
    """CS binding slots that are live aux outputs, the only ones CS.AUX_* answers for."""
    live = struct.unpack_from("<H", dev.pget("CS.STATUS", length=41), 4)[0]
    return {s for s in range(16) if live >> s & 1 and dev.pget("CS.BINDING", s, 24)[0] in AUX_TYPES}


def _get_reply_ok(chk, name, c, raw, label):
    if c["get_len"] == "var":
        if name == "MEAS.RTA_BANDS_ALL":   # no live channel is zero bytes, not an error (14.2)
            return chk.eq(len(raw) % 82, 0, f"{name}{label} whole 82-byte frames ({len(raw)} B)")
        return chk.ok(len(raw) > 0, f"{name}{label} variable reply is empty")
    return chk.eq(len(raw), c["get_len"], f"{name}{label} length")


def reply_len(c):
    if isinstance(c["get_len"], int):
        return c["get_len"]
    return 4096 if "BULK" in c["flags"] else 256


def _rta_idle(dev):
    return dev.pget("MEAS.RTA_STATUS", length=24)[1] == 0


def _keepalive_done(dev, was_idle):
    """KEEPALIVE GETs start an idle analyser (commands.md section 3, rule 2); put it back."""
    if was_idle:
        dev.pset("MEAS.RTA_STOP")


# --- Tests --------------------------------------------------------------------------------------

@test("crosscut")
def discovery_matches_registry(dev, profile, chk):
    """Every page's revision, command bitmaps and COMMAND_INFO (flags, sizes, address kind) match the registry."""
    require_paged(dev)
    doc, cmds = load_commands()
    page_list = dev.pget("SYS.PAGE_LIST", length=256)
    listed = dict(zip(page_list[0::2], page_list[1::2]))
    for pg in doc["pages"]:
        page, mn = pg["page"], pg["mnemonic"]
        want = {f"{mn}.{c['name']}": _expected(cmds[f"{mn}.{c['name']}"], profile) for c in pg["commands"]}
        if page not in listed:
            chk.ok(not any(want.values()), f"{mn} (0x{page:02X}) absent from PAGE_LIST but has unconditional commands")
            expect_stall(chk, lambda: dev.pget("SYS.PAGE_COMMANDS", page, 64), "VC_ERR_ADDRESS",
                         f"PAGE_COMMANDS of absent page {mn}")
            continue
        chk.eq(listed[page], pg["revision"], f"{mn} revision")
        bm = dev.pget("SYS.PAGE_COMMANDS", page, 64)
        got_get, got_set = bits(bm[:32]), bits(bm[32:])
        chk.eq(sorted((got_get | got_set) - {c["cmd"] for c in pg["commands"]}), [], f"{mn} bits the registry lacks")
        for name, exp in want.items():
            c = cmds[name]
            here = c["cmd"] in got_get or c["cmd"] in got_set
            if exp is not None:
                chk.eq(here, exp, f"{name} present (build {c['build'] or 'all'})")
            if not here:
                expect_stall(chk, lambda: dev.pget("SYS.COMMAND_INFO", c["id"], 8), "VC_ERR_ADDRESS",
                             f"COMMAND_INFO of absent {name}")
                continue
            chk.eq((c["cmd"] in got_get, c["cmd"] in got_set), (c["get"], c["set"]), f"{name} direction bits")
            info = command_info(dev, c["id"])
            want_len = 0xFFFF if c["get_len"] == "var" else (c["get_len"] or 0)
            chk.eq(info["flags"], sum(FLAG_BITS[f] for f in c["flags"]), f"{name} flags")
            chk.eq((info["get_len"], info["set_min"], info["set_max"]), (want_len, c["set_min"], c["set_max"]),
                   f"{name} sizes")
            chk.eq(ADDR_KINDS[info["kind"]] if info["kind"] < len(ADDR_KINDS) else info["kind"], c["addr"],
                   f"{name} address kind")
            chk.eq(info["dirs"], (1 if c["get"] else 0) | (2 if c["set"] else 0), f"{name} directions")
    chk.eq(sorted(set(listed) - {pg["page"] for pg in doc["pages"]}), [], "listed pages the registry lacks")


@test("crosscut", mutating=True)
def all_gets_respond(dev, profile, chk):
    """Every GET answers at a valid address with its registry length; the documented exceptions refuse as documented."""
    require_paged(dev)
    _doc, cmds = load_commands()
    aux = _aux_live(dev)
    was_idle = _rta_idle(dev)
    answered = 0
    try:
        for name, c in cmds.items():
            if not c["get"]:
                continue
            if not _present(dev, c, "GET"):
                # Absent only by its build condition (2.5), and then VC_ERR_UNKNOWN, never empty bytes.
                chk.ok(_expected(c, profile) is not True, f"{name} GET missing on a build that should have it")
                expect_stall(chk, lambda: dev.pget(name, length=8), "VC_ERR_UNKNOWN", f"GET of unbuilt {name}")
                continue
            addr = VALID_ADDR.get(name, 0)
            if name in AUX_GETS and addr not in aux:
                expect_stall(chk, lambda: dev.pget(name, addr), "VC_ERR_STATE",
                             f"{name} of slot {addr}, not a live aux output")
                continue
            try:
                raw = dev.pget(name, addr, reply_len(c))
            except PagedStall as s:
                chk.ok(False, f"{name} at 0x{addr:04X} refused: {s.result_name}")
                continue
            if _get_reply_ok(chk, name, c, raw, f" at 0x{addr:04X}"):
                answered += 1
    finally:
        _keepalive_done(dev, was_idle)
    chk.note(f"{answered} GETs answered; aux slots live: {sorted(aux) or 'none'}")


@test("crosscut")
def unknown_commands_refused(dev, profile, chk):
    """Unallocated pages, numbers no page defines and the reserved 0x00-0x0F are VC_ERR_UNKNOWN both ways."""
    require_paged(dev)
    _doc, cmds = load_commands()
    pc = dev.paged_commands()
    defined = {}
    for c in cmds.values():
        defined.setdefault(c["page"], set()).add(c["cmd"])
    for page in sorted(pc):
        free = [n for n in range(0x10, 0x100) if n not in defined.get(page, set())]
        step = max(1, len(free) // 6)
        probe = sorted({0x00, 0x01, 0x0F} | set(free[::step]) | {free[-1]})
        for n in probe:
            cid = (page << 8) | n
            expect_stall(chk, lambda: dev.pget(cid, length=8), "VC_ERR_UNKNOWN", f"GET 0x{cid:04X}")
            expect_stall(chk, lambda: dev.pset(cid), "VC_ERR_UNKNOWN", f"SET 0x{cid:04X}")
    unlisted = [p for p in range(0x01, 0x100) if p not in pc]
    for page in unlisted:
        expect_stall(chk, lambda: dev.pget((page << 8) | 0x10, length=8), "VC_ERR_UNKNOWN",
                     f"GET on unallocated page 0x{page:02X}")
    for page in (unlisted[0], 0x7F, 0x80, 0xE0, 0xEF, 0xFF):
        if page in unlisted:
            expect_stall(chk, lambda: dev.pset((page << 8) | 0x10, b"\x00"), "VC_ERR_UNKNOWN",
                         f"SET on unallocated page 0x{page:02X}")
    chk.note(f"{len(pc)} pages probed for free numbers, {len(unlisted)} unallocated pages")


@test("crosscut")
def wrong_direction_refused(dev, profile, chk):
    """A SET to every GET-only command and a GET to every SET-only command is VC_ERR_DIRECTION."""
    require_paged(dev)
    _doc, cmds = load_commands()
    for name, c in cmds.items():
        if c["get"] == c["set"] or not (_present(dev, c, "GET") or _present(dev, c, "SET")):
            continue
        info = command_info(dev, c["id"])
        if not chk.eq(info["dirs"], 1 if c["get"] else 2, f"{name} single direction on the device"):
            continue
        if c["get"]:
            expect_stall(chk, lambda: dev.pset(name), "VC_ERR_DIRECTION", f"SET to GET-only {name}")
        else:
            expect_stall(chk, lambda: dev.pget(name, length=8), "VC_ERR_DIRECTION", f"GET to SET-only {name}")


@test("crosscut")
def set_length_refused(dev, profile, chk):
    """Every non-bulk SET refuses one byte past its bounds each side, and 65 bytes, with VC_ERR_LENGTH; nothing changes."""
    require_paged(dev)
    _doc, cmds = load_commands()
    ranges = addr_ranges(profile, noun_count(dev))
    tested = 0
    for name, c in cmds.items():
        if not c["set"] or "BULK" in c["flags"] or not _present(dev, c, "SET"):
            continue
        info = command_info(dev, c["id"])
        if not chk.eq((info["set_min"], info["set_max"]), (c["set_min"], c["set_max"]), f"{name} SET bounds"):
            continue
        addr = 0 if c["addr"] in CMD_CHECKED else ranges[c["addr"]][0]
        lengths = [c["set_max"] + 1, MAX_SET + 1] + ([c["set_min"] - 1] if c["set_min"] else [])
        before = dev.pget(name, addr, reply_len(c)) if c["get"] and name not in AUX_GETS else None
        for n in lengths:
            assert_refused(c, info, n, addr, ranges)
            expect_stall(chk, lambda: dev.pset(name, bytes(n), addr), "VC_ERR_LENGTH", f"{n}-byte SET to {name}")
        if before is not None:
            chk.eq(dev.pget(name, addr, reply_len(c)), before, f"{name} unchanged by refused SETs")
        tested += 1
    chk.note(f"{tested} SET commands length-checked")


@test("crosscut", mutating=True)
def address_refused(dev, profile, chk):
    """The first wValues past each address kind's range are VC_ERR_ADDRESS both ways; the last valid one answers."""
    require_paged(dev)
    _doc, cmds = load_commands()
    ranges = addr_ranges(profile, noun_count(dev))
    aux = _aux_live(dev)
    was_idle = _rta_idle(dev)
    try:
        for name, c in cmds.items():
            if c["addr"] not in ranges or not (_present(dev, c, "GET") or _present(dev, c, "SET")):
                continue
            info = command_info(dev, c["id"])
            if not chk.eq(ADDR_KINDS[info["kind"]], c["addr"], f"{name} address kind on the device"):
                continue
            last, bad = ranges[c["addr"]]
            if c["get"] and name == "SYS.LAST_RESULT":
                # Its GETs are never recorded, refused ones included, so a read cannot hide the last reason (1.5).
                for a in bad:
                    before = dev.last_result()
                    chk.stalls(lambda: dev.pget(name, a, 8), f"GET {name} at 0x{a:04X}")
                    chk.eq(dev.last_result(), before, f"refused GET {name} at 0x{a:04X} left the record alone")
            elif c["get"]:
                for a in bad:
                    expect_stall(chk, lambda: dev.pget(name, a, reply_len(c)), "VC_ERR_ADDRESS",
                                 f"GET {name} at 0x{a:04X}")
                if name in AUX_GETS and last not in aux:
                    expect_stall(chk, lambda: dev.pget(name, last), "VC_ERR_STATE", f"{name} of non-aux slot {last}")
                elif name != "SYS.PAGE_COMMANDS":    # page 0xFF is valid to the dispatcher, not a real page
                    try:
                        _get_reply_ok(chk, name, c, dev.pget(name, last, reply_len(c)), f" at last 0x{last:04X}")
                    except PagedStall as s:
                        chk.ok(False, f"{name} at its last valid address 0x{last:04X} refused: {s.result_name}")
            if c["set"] and "BULK" not in c["flags"]:
                for a in bad:
                    assert_refused(c, info, c["set_min"], a, ranges)
                    expect_stall(chk, lambda: dev.pset(name, bytes(c["set_min"]), a), "VC_ERR_ADDRESS",
                                 f"SET {name} at 0x{a:04X}")
    finally:
        _keepalive_done(dev, was_idle)


@test("crosscut")
def quarantine_enter_bootloader(dev, profile, chk):
    """SYS.ENTER_BOOTLOADER never receives a request it could accept (it drops the device to BOOTSEL)."""
    raise Skip("excluded by policy: would re-enumerate device into BOOTSEL and end the session")
