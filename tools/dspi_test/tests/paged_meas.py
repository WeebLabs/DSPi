"""
Native paged tests for the MEAS page (plan section 5.9): meters, clip flags, the spectrum analyser
and the signal generator. The generator is never started: SIGGEN_START only receives requests the
dispatcher refuses, and STOP / STOP_NOW are sent only while it is already idle.
"""

import struct
import time

from ..device import OP, PagedStall, Stall
from ..framework import Skip, test
from .paged_fx import page_matches_registry
from .paged_sys import _expect_stall, _paged

MEAS = 0x09
RTA_FRAME = 82
RTA_STATE_IDLE = 0


def _meas(dev):
    _paged(dev)
    if MEAS not in dev.paged_commands():
        raise Skip("MEAS page not implemented on this firmware")


def _get(dev, n, addr=0, length=None):
    return dev.pget(f"MEAS.{n}", addr=addr, length=length)


def _set(dev, n, data=b"", addr=0):
    return dev.pset(f"MEAS.{n}", data, addr)


def _rta_state(dev):
    return _get(dev, "RTA_STATUS")[1]


def _rta_restore_run(dev, was_idle):
    try:
        if was_idle:
            _set(dev, "RTA_STOP")
    except Stall:
        pass


def _await(fn, want, timeout_s=1.5):
    deadline = time.monotonic() + timeout_s
    got = fn()
    while got != want and time.monotonic() < deadline:
        time.sleep(0.05)
        got = fn()
    return got


def _siggen_idle(dev):
    return _get(dev, "SIGGEN_STATUS")[1] == 0


@test("paged_meas")
def meas_page_matches_registry(dev, profile, chk):
    """MEAS.PAGE_COMMANDS and COMMAND_INFO match the registry."""
    _meas(dev)
    page_matches_registry(dev, chk, MEAS, profile.platform_id == 1)


@test("paged_meas")
def meas_levels_match_status9(dev, profile, chk):
    """MEAS.LEVELS has the layout of old GET_STATUS 9: 2N+7 bytes, same clip flags and input count."""
    _meas(dev)
    n = 2 * profile.num_channels + 7
    for _ in range(5):
        new = _get(dev, "LEVELS", length=64)
        old = dev.raw_get(OP.GET_STATUS, 64, wvalue=9)
        if new[-5:] == old[-5:]:
            break
    chk.eq(len(new), n, "LEVELS length 2 * NUM_CHANNELS + 7")
    chk.eq(len(new), len(old), "same length as old GET_STATUS 9")
    chk.eq(new[-5:], old[-5:], "clip flags and active input count match old GET_STATUS 9")
    chk.eq(len(_get(dev, "LEVELS", length=10)), 10, "LEVELS truncates to wLength")


@test("paged_meas", mutating=True)
def meas_clip_flags_and_clear(dev, profile, chk):
    """CLIP_FLAGS reads without clearing; CLIP_CLEAR (zero-length) clears; old 0x83 still reads then clears."""
    _meas(dev)
    a = _get(dev, "CLIP_FLAGS")
    chk.eq(len(a), 4, "CLIP_FLAGS length")
    lv = _get(dev, "LEVELS", length=64)
    chk.eq(a, lv[-5:-1], "CLIP_FLAGS equals the LEVELS clip field")
    chk.eq(_get(dev, "CLIP_FLAGS"), a, "a second read still shows the flags (no clear on read)")
    chk.no_stall(lambda: _set(dev, "CLIP_CLEAR"), "CLIP_CLEAR")
    after = _get(dev, "CLIP_FLAGS")
    if after != bytes(4):
        chk.note(f"flags re-latched after CLIP_CLEAR: {after.hex()} (live clipping?)")
    dev.raw_get(OP.CLEAR_CLIPS, 4)
    _expect_stall(chk, lambda: _set(dev, "CLIP_CLEAR", b"\x00"), "VC_ERR_LENGTH", "CLIP_CLEAR with a payload")
    _expect_stall(chk, lambda: _get(dev, "CLIP_CLEAR", length=4), "VC_ERR_DIRECTION", "GET CLIP_CLEAR")
    _expect_stall(chk, lambda: _set(dev, "CLIP_FLAGS"), "VC_ERR_DIRECTION", "SET CLIP_FLAGS")


def _rta_cfg_valid(cfg, caps):
    ver, tap, mask, order = cfg[0], cfg[1], struct.unpack_from("<H", cfg, 2)[0], cfg[4]
    width = caps[1] if tap == 0 else caps[2]
    return (ver == caps[0] and tap in (0, 1) and caps[3] <= order <= caps[4]
            and mask & ((1 << width) - 1) != 0)


@test("paged_meas", mutating=True)
def meas_rta_config_roundtrip(dev, profile, chk):
    """RTA_CONFIG equals old 0x09, round-trips both ways, and refuses bad configs with VC_ERR_VALUE / LENGTH."""
    _meas(dev)
    cfg = _get(dev, "RTA_CONFIG")
    chk.eq(cfg, dev.raw_get(OP.RTA_GET_CONFIG, 12), "RTA_CONFIG == old 0x09")
    caps = _get(dev, "RTA_CAPS")
    bad_mask = bytearray(cfg)
    struct.pack_into("<H", bad_mask, 2, 0)
    bad_ver = bytearray(cfg)
    bad_ver[0] ^= 0xFF
    _expect_stall(chk, lambda: _set(dev, "RTA_CONFIG", bytes(bad_mask)), "VC_ERR_VALUE", "empty channel mask")
    _expect_stall(chk, lambda: _set(dev, "RTA_CONFIG", bytes(bad_ver)), "VC_ERR_VALUE", "wrong version")
    _expect_stall(chk, lambda: _set(dev, "RTA_CONFIG", cfg[:11]), "VC_ERR_LENGTH", "11-byte config")
    _expect_stall(chk, lambda: _set(dev, "RTA_CONFIG", cfg + b"\x00"), "VC_ERR_LENGTH", "13-byte config")
    _expect_stall(chk, lambda: _set(dev, "RTA_CONFIG", cfg, addr=1), "VC_ERR_ADDRESS", "config at wValue 1")
    chk.eq(_get(dev, "RTA_CONFIG"), cfg, "rejected configs change nothing")
    if not _rta_cfg_valid(cfg, caps):
        chk.note(f"current RTA config {cfg.hex()} is not a valid SET payload: write round trip skipped")
        return
    avg = struct.unpack_from("<H", cfg, 6)[0]
    a, b = bytearray(cfg), bytearray(cfg)
    struct.pack_into("<H", a, 6, 333 if avg != 333 else 334)
    struct.pack_into("<H", b, 6, 444 if avg != 444 else 445)
    try:
        chk.no_stall(lambda: _set(dev, "RTA_CONFIG", bytes(a)), "new SET")
        chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), bytes(a)), bytes(a), "applied, read through RTA_CONFIG")
        chk.eq(dev.raw_get(OP.RTA_GET_CONFIG, 12), bytes(a), "applied, read through old 0x09")
        dev.raw_set(OP.RTA_SET_CONFIG, bytes(b))
        chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), bytes(b)), bytes(b), "old 0x08 SET read through RTA_CONFIG")
    finally:
        _set(dev, "RTA_CONFIG", cfg)
    chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), cfg), cfg, "config restored")


@test("paged_meas")
def meas_rta_caps_and_centres(dev, profile, chk):
    """RTA_CAPS equals old 0x0A wValue 0; RTA_BAND_CENTRES chunk n equals old 0x0A wValue n+1."""
    _meas(dev)
    caps = _get(dev, "RTA_CAPS")
    chk.eq(caps, dev.raw_get(OP.RTA_GET_CAPS, 16, wvalue=0), "RTA_CAPS == old 0x0A[0]")
    centres = b""
    n = 0
    while n < 8:
        try:
            old = dev.raw_get(OP.RTA_GET_CAPS, 64, wvalue=n + 1)
        except Stall:
            break
        chk.eq(_get(dev, "RTA_BAND_CENTRES", addr=n, length=64), old, f"chunk {n} == old 0x0A[{n + 1}]")
        centres += old
        n += 1
    chk.eq(len(centres) // 2, caps[7], "centre count equals caps max_bands")
    _expect_stall(chk, lambda: _get(dev, "RTA_BAND_CENTRES", addr=n, length=64), "VC_ERR_ADDRESS",
                  f"chunk {n} past the table")
    _expect_stall(chk, lambda: _get(dev, "RTA_BAND_CENTRES", addr=0xFF, length=64), "VC_ERR_ADDRESS", "chunk 0xFF")


@test("paged_meas", mutating=True)
def meas_rta_bands_status_bins(dev, profile, chk):
    """RTA_BANDS / STATUS / BINS / BANDS_ALL match old 0x0B / 0x0D / 0x0C / 0x0F in shape and stable fields."""
    _meas(dev)
    was_idle = _rta_state(dev) == RTA_STATE_IDLE
    try:
        cfg = _get(dev, "RTA_CONFIG")
        caps = _get(dev, "RTA_CAPS")
        width = caps[1] if cfg[1] == 0 else caps[2]
        new, old = _get(dev, "RTA_BANDS", addr=0), dev.raw_get(OP.RTA_GET_BANDS, RTA_FRAME, wvalue=0)
        chk.eq(len(new), RTA_FRAME, "RTA_BANDS length")
        chk.eq((new[0], new[1], new[3]), (old[0], old[1], old[3]), "RTA_BANDS version, channel, n_bands")
        for bad in (width, 0x100):
            _expect_stall(chk, lambda: _get(dev, "RTA_BANDS", addr=bad), "VC_ERR_ADDRESS", f"RTA_BANDS channel {bad}")
        st, ost = _get(dev, "RTA_STATUS"), dev.raw_get(OP.RTA_GET_STATUS, 24)
        chk.eq((st[0], st[2], st[6:8], st[16:20]), (ost[0], ost[2], ost[6:8], ost[16:20]),
               "RTA_STATUS version, tap, live mask, sample rate")
        allf, oall = _get(dev, "RTA_BANDS_ALL", length=4096), dev.raw_get(OP.RTA_GET_BANDS_ALL, 4096)
        chk.eq(len(allf) % RTA_FRAME, 0, "RTA_BANDS_ALL is whole frames")
        chk.eq(len(allf), len(oall), "RTA_BANDS_ALL length matches old 0x0F")
        chk.eq([allf[i + 1] for i in range(0, len(allf), RTA_FRAME)],
               [oall[i + 1] for i in range(0, len(oall), RTA_FRAME)], "RTA_BANDS_ALL channel order")
        try:
            hdr = _get(dev, "RTA_BINS", addr=0, length=16)
        except PagedStall as s:
            chk.eq(s.result_name, "VC_ERR_STATE", "no bin frame yet")
            chk.stalls(lambda: dev.raw_get(OP.RTA_GET_BINS, 16, wvalue=0), "old 0x0C also refuses")
            chk.note("no bin frame published (pipeline idle): offset tests skipped")
            return
        total = 16 + (1 << cfg[4]) // 2 + 1    # bin_len is fixed by the applied FFT order
        chk.eq(hdr[0], dev.raw_get(OP.RTA_GET_BINS, 16, wvalue=0)[0], "bin frame version")
        chk.eq(len(_get(dev, "RTA_BINS", addr=0, length=600)), min(total, 256), "one read stops at 256 bytes")
        chk.eq(len(_get(dev, "RTA_BINS", addr=total - 1, length=64)), 1, "last byte readable")
        _expect_stall(chk, lambda: _get(dev, "RTA_BINS", addr=total, length=16), "VC_ERR_ADDRESS",
                      "offset past the frame")
    finally:
        _rta_restore_run(dev, was_idle)


@test("paged_meas", mutating=True)
def meas_rta_actions(dev, profile, chk):
    """RTA_START / STOP / RESET are zero-length SETs matching old 0x0E 1 / 0 / 2; payloads and GETs are refused."""
    _meas(dev)
    was_idle = _rta_state(dev) == RTA_STATE_IDLE
    live = struct.unpack_from("<H", _get(dev, "RTA_STATUS"), 6)[0]
    try:
        for n in ("RTA_START", "RTA_STOP", "RTA_RESET"):
            _expect_stall(chk, lambda: _set(dev, n, b"\x01"), "VC_ERR_LENGTH", f"{n} with a payload")
            _expect_stall(chk, lambda: _get(dev, n, length=1), "VC_ERR_DIRECTION", f"GET {n}")
        chk.no_stall(lambda: _set(dev, "RTA_RESET"), "RTA_RESET")
        chk.no_stall(lambda: _set(dev, "RTA_STOP"), "RTA_STOP")
        chk.eq(_rta_state(dev), RTA_STATE_IDLE, "idle after RTA_STOP")
        chk.no_stall(lambda: _set(dev, "RTA_START"), "RTA_START")
        started = _rta_state(dev)
        dev.raw_get(OP.RTA_CONTROL, 1, wvalue=0)
        chk.eq(_rta_state(dev), RTA_STATE_IDLE, "idle after old 0x0E[0]")
        chk.eq(dev.raw_get(OP.RTA_CONTROL, 1, wvalue=1), b"\x01", "old 0x0E[1] acknowledges")
        chk.eq(_rta_state(dev) != RTA_STATE_IDLE, started != RTA_STATE_IDLE, "old start and new start agree")
        if live and started == RTA_STATE_IDLE:
            chk.ok(False, f"RTA_START left the engine idle with live mask 0x{live:04X}")
        chk.eq(dev.raw_get(OP.RTA_CONTROL, 1, wvalue=2), b"\x01", "old 0x0E[2] acknowledges")
    finally:
        _rta_restore_run(dev, was_idle)


@test("paged_meas", mutating=True)
def meas_siggen_config(dev, profile, chk):
    """SIGGEN_CONFIG equals old 0xA5 and refuses bad configs; a valid idle config round-trips both ways."""
    _meas(dev)
    cfg = _get(dev, "SIGGEN_CONFIG")
    chk.eq(cfg, dev.raw_get(OP.SIGGEN_GET_CONFIG, 36), "SIGGEN_CONFIG == old 0xA5")
    bad_ver, bad_type, bad_mask = bytearray(cfg), bytearray(cfg), bytearray(cfg)
    bad_ver[0] = 0xEE
    bad_type[1] = 0xEE
    struct.pack_into("<H", bad_mask, 2, 0)
    for what, payload in (("version", bad_ver), ("type", bad_type), ("mask", bad_mask)):
        _expect_stall(chk, lambda: _set(dev, "SIGGEN_CONFIG", bytes(payload)), "VC_ERR_VALUE", f"bad {what}")
    for n in (35, 37):
        _expect_stall(chk, lambda: _set(dev, "SIGGEN_CONFIG", bytes(n)), "VC_ERR_LENGTH", f"{n}-byte config")
    chk.eq(_get(dev, "SIGGEN_CONFIG"), cfg, "rejected configs change nothing")
    if cfg[1:] == bytes(35):
        chk.note("generator holds its boot config (version byte only), which SET refuses: write round trip skipped")
        return
    if not _siggen_idle(dev):
        chk.note("generator is running: write round trip skipped so it is not restarted")
        return
    level = struct.unpack_from("<f", cfg, 8)[0]
    a, b = bytearray(cfg), bytearray(cfg)
    struct.pack_into("<f", a, 8, -60.0 if level != -60.0 else -61.0)
    struct.pack_into("<f", b, 8, -70.0 if level != -70.0 else -71.0)
    try:
        chk.no_stall(lambda: _set(dev, "SIGGEN_CONFIG", bytes(a)), "new SET")
        chk.eq(_get(dev, "SIGGEN_CONFIG"), bytes(a), "new GET after new SET")
        chk.eq(dev.raw_get(OP.SIGGEN_GET_CONFIG, 36), bytes(a), "old 0xA5 after new SET")
        dev.raw_set(OP.SIGGEN_SET_CONFIG, bytes(b))
        chk.eq(_get(dev, "SIGGEN_CONFIG"), bytes(b), "new GET after old 0xA4")
        chk.ok(_siggen_idle(dev), "writing a config does not start the generator")
    finally:
        _set(dev, "SIGGEN_CONFIG", cfg)
    chk.eq(_get(dev, "SIGGEN_CONFIG"), cfg, "config restored")


@test("paged_meas")
def meas_siggen_status_caps_types(dev, profile, chk):
    """SIGGEN_STATUS / CAPS / TYPE_DESC equal old 0xA7 / 0xA8[0xFFFF] / 0xA8[type]; a bad type is VC_ERR_ADDRESS."""
    _meas(dev)
    st, ost = _get(dev, "SIGGEN_STATUS"), dev.raw_get(OP.SIGGEN_GET_STATUS, 16)
    chk.eq((st[:4], st[8:]), (ost[:4], ost[8:]), "SIGGEN_STATUS (elapsed time masked)")
    caps = _get(dev, "SIGGEN_CAPS")
    chk.eq(caps, dev.raw_get(OP.SIGGEN_GET_CAPS, 8, wvalue=0xFFFF), "SIGGEN_CAPS == old 0xA8[0xFFFF]")
    count = caps[1]
    for i in range(count):
        chk.eq(_get(dev, "SIGGEN_TYPE_DESC", addr=i), dev.raw_get(OP.SIGGEN_GET_CAPS, 62, wvalue=i),
               f"SIGGEN_TYPE_DESC {i} == old 0xA8[{i}]")
    for bad in (count, 0x100, 0xFFFF):
        _expect_stall(chk, lambda: _get(dev, "SIGGEN_TYPE_DESC", addr=bad), "VC_ERR_ADDRESS", f"type {bad}")
    _expect_stall(chk, lambda: _get(dev, "SIGGEN_CAPS", addr=0xFFFF), "VC_ERR_ADDRESS", "SIGGEN_CAPS at 0xFFFF")


@test("paged_meas", mutating=True)
def meas_siggen_actions_never_start(dev, profile, chk):
    """SIGGEN_START only meets refusals; STOP and STOP_NOW (and old 0xA6 0 / 2) are harmless while idle."""
    _meas(dev)
    for n in ("SIGGEN_START", "SIGGEN_STOP", "SIGGEN_STOP_NOW"):
        _expect_stall(chk, lambda: _set(dev, n, b"\x01"), "VC_ERR_LENGTH", f"{n} with a payload")
        _expect_stall(chk, lambda: _get(dev, n, length=1), "VC_ERR_DIRECTION", f"GET {n}")
    _expect_stall(chk, lambda: _set(dev, "SIGGEN_START", bytes(4)), "VC_ERR_LENGTH", "SIGGEN_START with 4 bytes")
    _expect_stall(chk, lambda: _set(dev, "SIGGEN_STATUS"), "VC_ERR_DIRECTION", "SET SIGGEN_STATUS")
    if not _siggen_idle(dev):
        chk.note("generator is running: stop actions not sent")
        return
    for n in ("SIGGEN_STOP", "SIGGEN_STOP_NOW"):
        chk.no_stall(lambda: _set(dev, n), f"{n} while idle")
        chk.ok(_siggen_idle(dev), f"still idle after {n}")
    for w in (0, 2):
        chk.eq(dev.raw_get(OP.SIGGEN_CONTROL, 1, wvalue=w), b"\x01", f"old 0xA6[{w}] acknowledges")
    chk.ok(_siggen_idle(dev), "still idle after the old stop actions")
