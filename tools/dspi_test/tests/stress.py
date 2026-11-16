"""
Stress and fuzz on a registry-chosen safe set: rapid valid SETs, then GETs and SETs with fuzzed
addresses, lengths and payloads. The device must stay responsive and answer each request as predicted.
Scale with --stress N. The global snapshot restore puts every changed live parameter back.
"""

import random

from ..device import Disconnected, PagedStall
from ..framework import Skip, test
from ..actions import live_param_actions
from ..helpers import require_paged
from .crosscut import (ADDR_KINDS, NEVER_VALID, addr_ranges, addr_valid, assert_refused, command_info,
                       load_commands, noun_count, reply_len)

UNSAFE_FLAGS = {"DEFERRED", "FLASH", "RESTART", "BULK", "USB_ONLY"}
# Valid-length SETs to these carry random bytes: any value is clamped or refused (commands.md 10.5,
# 12), so none can raise the output level without bound. Every other command gets its own GET back.
RANDOM_CONTENT_OK = frozenset({
    "ROUTE.MASTER_VOLUME", "ROUTE.USER_VOLUME", "ROUTE.OUTPUT_DELAY", "ROUTE.CHANNEL_DELAY",
    "DYN.LEVELLER_AMOUNT", "DYN.LEVELLER_SPEED", "DYN.LEVELLER_MAX_GAIN", "DYN.LEVELLER_LOOKAHEAD",
    "DYN.LEVELLER_GATE", "DYN.LOUDNESS_ENABLE", "DYN.LOUDNESS_REF", "DYN.LOUDNESS_INTENSITY",
    "DYN.LIMITER_ENABLE", "DYN.LIMITER_THRESHOLD", "DYN.LIMITER_RELEASE", "DYN.LIMITER_LINK_GROUP",
})
# live_param_actions() name -> the command it writes.
ACTION_CMD = {
    "master_volume": "ROUTE.MASTER_VOLUME", "user_volume": "ROUTE.USER_VOLUME", "preamp": "ROUTE.PREAMP",
    "loudness_ref": "DYN.LOUDNESS_REF", "loudness_intensity": "DYN.LOUDNESS_INTENSITY",
    "crossfeed_freq": "FX.CROSSFEED_FREQ", "crossfeed_feed": "FX.CROSSFEED_FEED",
    "leveller_amount": "DYN.LEVELLER_AMOUNT", "leveller_max_gain": "DYN.LEVELLER_MAX_GAIN",
    "leveller_gate": "DYN.LEVELLER_GATE", "leveller_speed": "DYN.LEVELLER_SPEED",
    "output_gain": "ROUTE.OUTPUT_GAIN", "output_delay": "ROUTE.OUTPUT_DELAY", "eq": "FILTER.BAND",
    "matrix": "ROUTE.MATRIX_ROUTE",
}
GET_LENGTHS = (1, 2, 4, 8, 16, 24, 32, 64, 256)


def _iters(profile, default):
    n = getattr(profile, "stress_iters", 0) or 0
    return max(n, default)


def _safe(c, profile_present):
    """The stress rule: a read-write live parameter with no deferred, flash or restart work."""
    return (profile_present and c["get"] and c["set"] and not (c["flags"] & UNSAFE_FLAGS)
            and c["name_full"] not in NEVER_VALID and c["set_min"] == c["set_max"] == c["get_len"])


def _safe_cmds(dev, ranges):
    _doc, cmds = load_commands()
    pc = dev.paged_commands()
    out = []
    for name, c in cmds.items():
        c = dict(c, name_full=name)
        gets, sets = pc.get(c["page"], (frozenset(), frozenset()))
        if c["addr"] in ranges and _safe(c, c["cmd"] in gets and c["cmd"] in sets):
            out.append(c)
    return out


def _random_addr(rng, kind, ranges):
    last, bad = ranges[kind]
    pick = rng.randrange(3)
    if pick == 0:
        return rng.randrange(0x10000)
    if pick == 1:
        return rng.choice(bad)
    if kind == "CH_BAND":
        return (rng.randrange((last >> 8) + 1) << 8) | rng.randrange(10)
    if kind in ("IN_OUT", "CS_MACRO_STEP"):
        return (rng.randrange((last >> 8) + 1) << 8) | rng.randrange((last & 0xFF) + 1)
    return rng.randrange(last + 1)


def _lg_off(dev):
    lg = dev.pget("INPUT.LG_SOUND_SYNC_ENABLE", length=1)
    dev.pset_u8("INPUT.LG_SOUND_SYNC_ENABLE", 0)
    return lg


@test("stress", mutating=True)
def stress_random_valid_sets(dev, profile, chk):
    """A storm of random valid SETs over the rule-chosen live parameters; the device stays coherent."""
    require_paged(dev)
    _doc, cmds = load_commands()
    rng = random.Random(0xD597)
    iters = _iters(profile, 60)
    actions, dropped = [], []
    for name, fn in live_param_actions(dev, profile, rng):
        c = cmds[ACTION_CMD[name]]
        (dropped if c["flags"] & UNSAFE_FLAGS or ACTION_CMD[name] in NEVER_VALID else actions).append((name, fn))
    if not actions:
        raise Skip("no live parameter action passes the stress rule")
    lg = _lg_off(dev)
    try:
        for i in range(iters):
            rng.choice(actions)[1]()
            if i % 12 == 0 and not dev.is_alive():
                chk.ok(False, f"device unresponsive mid-storm at iter {i}")
                return
        chk.ok(dev.is_alive(), "device responsive after random-set storm")
        chk.in_range(dev.pget_f32("ROUTE.MASTER_VOLUME"), -128.0, 0.0, "master vol sane after storm")
        chk.note(f"{iters} random valid SETs survived; left out by the rule: {[n for n, _ in dropped]}")
    finally:
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", lg)


@test("stress", mutating=True)
def stress_get_fuzz(dev, profile, chk):
    """Every non-bulk GET with random wValue and wLength: data no longer than asked, or the predicted refusal."""
    require_paged(dev)
    _doc, cmds = load_commands()
    ranges = addr_ranges(profile, noun_count(dev))
    pc = dev.paged_commands()
    reads = [(n, c) for n, c in cmds.items() if c["get"] and "BULK" not in c["flags"]
             and c["cmd"] in pc.get(c["page"], ((), ()))[0]]
    rng = random.Random(0x6A11)
    iters = _iters(profile, 200)
    was_idle = dev.pget("MEAS.RTA_STATUS", length=24)[1] == 0
    try:
        for i in range(iters):
            name, c = rng.choice(reads)
            addr = rng.randrange(0x10000)
            ln = rng.choice(GET_LENGTHS)
            try:
                r = dev.pget(name, addr, ln)
                chk.ok(0 < len(r) <= ln, f"{name} at 0x{addr:04X}: {len(r)} B for wLength {ln}")
                if c["addr"] in ranges:
                    chk.ok(addr_valid(c["addr"], addr, ranges), f"{name} answered out-of-range 0x{addr:04X}")
            except PagedStall as s:
                if c["addr"] in ranges and not addr_valid(c["addr"], addr, ranges):
                    chk.eq(s.result_name, "VC_ERR_ADDRESS", f"{name} at 0x{addr:04X}")
                else:
                    chk.member(s.result_name, ("VC_ERR_ADDRESS", "VC_ERR_STATE"), f"{name} at 0x{addr:04X}")
            except Disconnected:
                raise
            if i % 40 == 0 and not dev.is_alive():
                chk.ok(False, f"device unresponsive during GET fuzz at iter {i}")
                return
        chk.ok(dev.is_alive(), "device responsive after GET fuzz")
        chk.note(f"{iters} fuzzed GETs over {len(reads)} commands survived")
    finally:
        if was_idle:
            dev.pset("MEAS.RTA_STOP")


@test("stress", mutating=True)
def stress_payload_fuzz(dev, profile, chk):
    """Rule-chosen SETs with random address, length and content: each gets the predicted result, never a crash."""
    require_paged(dev)
    ranges = addr_ranges(profile, noun_count(dev))
    safe = _safe_cmds(dev, ranges)
    if not safe:
        raise Skip("no command passes the stress rule")
    infos = {c["id"]: command_info(dev, c["id"]) for c in safe}
    for c in safe:
        chk.eq(ADDR_KINDS[infos[c["id"]]["kind"]], c["addr"], f"{c['name_full']} address kind on the device")
    rng = random.Random(0x9E37)
    iters = _iters(profile, 150)
    counts = {}
    lg = _lg_off(dev)
    try:
        for i in range(iters):
            c = rng.choice(safe)
            name, n = c["name_full"], c["set_min"]
            addr = _random_addr(rng, c["addr"], ranges)
            if c["addr"] == "OUTPUT_ALL" and addr == 0xFF:
                continue            # valid on a SET, but there is no GET of "all" to write back
            ln = n if rng.randrange(2) else rng.randrange(0, 65)
            ok_addr = addr_valid(c["addr"], addr, ranges, is_get=False)
            if ln == n and ok_addr:
                payload = (bytes(rng.randrange(256) for _ in range(n)) if name in RANDOM_CONTENT_OK
                           else dev.pget(name, addr, reply_len(c)))
                want = ("VC_OK", "VC_ERR_VALUE") if name in RANDOM_CONTENT_OK else ("VC_OK",)
            else:
                assert_refused(c, infos[c["id"]], ln, addr, ranges)
                payload = bytes(rng.randrange(256) for _ in range(ln))
                want = ("VC_ERR_LENGTH",) if ln != n else ("VC_ERR_ADDRESS",)
            try:
                dev.pset(name, payload, addr)
                got = "VC_OK"
            except PagedStall as s:
                got = s.result_name or "STALL"
            except Disconnected:
                raise
            chk.member(got, want, f"{name} {ln} B at 0x{addr:04X}")
            counts[got] = counts.get(got, 0) + 1
            if i % 30 == 0 and not dev.is_alive():
                chk.ok(False, f"device unresponsive during payload fuzz at iter {i}")
                return
        chk.ok(dev.is_alive(), "device responsive after payload fuzz")
        chk.note(f"{iters} fuzzed SETs over {len(safe)} commands: {counts}")
    finally:
        dev.pset("INPUT.LG_SOUND_SYNC_ENABLE", lg)
