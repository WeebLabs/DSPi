"""
Native paged tests for the MEAS page (plan section 5.9): meters, clip flags, the spectrum analyser
and the signal generator. The generator is never started: SIGGEN_START only receives requests the
dispatcher refuses, and STOP / STOP_NOW are sent only while it is already idle.
"""

import math
import struct
import time

from ..device import PagedStall, Stall
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
def meas_levels_shape(dev, profile, chk):
    """MEAS.LEVELS is 2N+7 bytes; its clip field equals CLIP_FLAGS and its input count equals INPUT.STATUS."""
    _meas(dev)
    n = 2 * profile.num_channels + 7
    for _ in range(5):
        lv = _get(dev, "LEVELS", length=64)
        clips = _get(dev, "CLIP_FLAGS")
        if lv[-5:-1] == clips:
            break
    chk.eq(len(lv), n, "LEVELS length 2 * NUM_CHANNELS + 7")
    chk.eq(lv[-5:-1], clips, "LEVELS clip field == CLIP_FLAGS")
    if 0x03 in dev.paged_commands():
        chk.eq(lv[-1], dev.pget("INPUT.STATUS", length=12)[1], "LEVELS active input count == INPUT.STATUS byte 1")
    chk.eq(len(_get(dev, "LEVELS", length=10)), 10, "LEVELS truncates to wLength")


@test("paged_meas", mutating=True)
def meas_clip_flags_and_clear(dev, profile, chk):
    """CLIP_FLAGS reads without clearing; CLIP_CLEAR (zero-length) clears; payloads and wrong directions are refused."""
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
    """RTA_CONFIG is 12 bytes in the caps' version, round-trips, and refuses bad configs with VC_ERR_VALUE / LENGTH."""
    _meas(dev)
    cfg = _get(dev, "RTA_CONFIG")
    caps = _get(dev, "RTA_CAPS")
    chk.eq(len(cfg), 12, "RTA_CONFIG length")
    chk.eq(cfg[0], caps[0], "RTA_CONFIG version == RTA_CAPS version")
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
        chk.no_stall(lambda: _set(dev, "RTA_CONFIG", bytes(a)), "first SET")
        chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), bytes(a)), bytes(a), "first SET applied")
        chk.no_stall(lambda: _set(dev, "RTA_CONFIG", bytes(b)), "second SET")
        chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), bytes(b)), bytes(b), "second SET applied")
    finally:
        _set(dev, "RTA_CONFIG", cfg)
    chk.eq(_await(lambda: _get(dev, "RTA_CONFIG"), cfg), cfg, "config restored")


@test("paged_meas")
def meas_rta_caps_and_centres(dev, profile, chk):
    """RTA_CAPS agrees with the profile; RTA_BAND_CENTRES chunks hold max_bands ascending centres, then VC_ERR_ADDRESS."""
    _meas(dev)
    caps = _get(dev, "RTA_CAPS")
    chk.eq(len(caps), 16, "RTA_CAPS length")
    chk.eq((caps[1], caps[2]), (profile.num_input_channels, profile.num_output_channels), "tap widths")
    chk.ok(caps[3] <= caps[5] <= caps[4], f"FFT order min <= default <= max: {caps[3:6].hex()}")
    chk.ok(caps[6] <= caps[7], f"bass bands within max bands: {caps[6]}/{caps[7]}")
    centres = b""
    n = 0
    while n < 8:
        try:
            chunk = _get(dev, "RTA_BAND_CENTRES", addr=n, length=64)
        except PagedStall as s:
            chk.eq(s.result_name, "VC_ERR_ADDRESS", f"chunk {n} past the table")
            break
        chk.ok(0 < len(chunk) <= 64 and len(chunk) % 2 == 0, f"chunk {n} length {len(chunk)}")
        centres += chunk
        n += 1
    chk.eq(len(centres) // 2, caps[7], "centre count equals caps max_bands")
    hz = list(struct.unpack(f"<{len(centres) // 2}H", centres))
    chk.ok(all(a < b for a, b in zip(hz, hz[1:])), f"centres ascend: {hz[:4]}...{hz[-4:]}")
    _expect_stall(chk, lambda: _get(dev, "RTA_BAND_CENTRES", addr=n, length=64), "VC_ERR_ADDRESS",
                  f"chunk {n} past the table")
    _expect_stall(chk, lambda: _get(dev, "RTA_BAND_CENTRES", addr=0xFF, length=64), "VC_ERR_ADDRESS", "chunk 0xFF")


@test("paged_meas", mutating=True)
def meas_rta_bands_status_bins(dev, profile, chk):
    """RTA_BANDS / STATUS / BINS / BANDS_ALL shapes agree with RTA_CONFIG, the live mask and SYS.SYSTEM_STATUS."""
    _meas(dev)
    was_idle = _rta_state(dev) == RTA_STATE_IDLE
    try:
        cfg = _get(dev, "RTA_CONFIG")
        caps = _get(dev, "RTA_CAPS")
        width = caps[1] if cfg[1] == 0 else caps[2]
        bands = _get(dev, "RTA_BANDS", addr=0)
        chk.eq(len(bands), RTA_FRAME, "RTA_BANDS length")
        chk.eq(bands[1], 0, "RTA_BANDS echoes the channel")
        chk.ok(bands[3] <= caps[7], f"RTA_BANDS n_bands {bands[3]} within max {caps[7]}")
        chk.eq(bands[6:8], b"\0\0", "RTA_BANDS reserved")
        for bad in (width, 0x100):
            _expect_stall(chk, lambda: _get(dev, "RTA_BANDS", addr=bad), "VC_ERR_ADDRESS", f"RTA_BANDS channel {bad}")
        st = _get(dev, "RTA_STATUS")
        chk.eq(len(st), 24, "RTA_STATUS length")
        chk.eq(st[2], cfg[1], "RTA_STATUS tap == RTA_CONFIG tap")
        live = struct.unpack_from("<H", st, 6)[0]
        chk.eq(st[5], bin(live).count("1"), "live_count == popcount(live_mask)")
        chk.eq(live & ~((1 << width) - 1), 0, "no live channel past the tap width")
        if 0x01 in dev.paged_commands():
            sys_rate = struct.unpack_from("<I", dev.pget("SYS.SYSTEM_STATUS"), 4)[0]
            chk.eq(struct.unpack_from("<I", st, 16)[0], sys_rate, "RTA_STATUS sample rate == SYS.SYSTEM_STATUS")
        for _ in range(3):
            allf = _get(dev, "RTA_BANDS_ALL", length=4096)
            live = struct.unpack_from("<H", _get(dev, "RTA_STATUS"), 6)[0]
            chans = [allf[i + 1] for i in range(0, len(allf), RTA_FRAME)]
            if chans == [c for c in range(16) if live >> c & 1]:
                break
        chk.eq(len(allf) % RTA_FRAME, 0, "RTA_BANDS_ALL is whole frames")
        chk.eq(chans, [c for c in range(16) if live >> c & 1], "RTA_BANDS_ALL has one frame per live channel, in order")
        try:
            hdr = _get(dev, "RTA_BINS", addr=0, length=16)
        except PagedStall as s:
            chk.eq(s.result_name, "VC_ERR_STATE", "no bin frame yet")
            chk.note("no bin frame published (pipeline idle): offset tests skipped")
            return
        nbins = (1 << cfg[4]) // 2
        total = 16 + nbins + 1    # bin_len is fixed by the applied FFT order
        if hdr[0] == 0:
            # The layout is sized at config time; the header stays zero until the first transform.
            chk.eq(hdr, bytes(16), "an unpublished bin frame has an all-zero header")
            chk.eq(_get(dev, "RTA_BINS", addr=total - 1, length=1), b"\xff", "and its seq tail reads 0xFF")
            chk.note("bin frame not yet published (no transform since the config): header fields not checked")
        else:
            chk.eq(hdr[3], cfg[4], "bin frame FFT order == RTA_CONFIG")
            chk.eq(struct.unpack_from("<H", hdr, 8)[0], nbins, "bin frame n_bins")
        chk.eq(len(_get(dev, "RTA_BINS", addr=0, length=600)), min(total, 256), "one read stops at 256 bytes")
        chk.eq(len(_get(dev, "RTA_BINS", addr=total - 1, length=64)), 1, "last byte readable")
        _expect_stall(chk, lambda: _get(dev, "RTA_BINS", addr=total, length=16), "VC_ERR_ADDRESS",
                      "offset past the frame")
    finally:
        _rta_restore_run(dev, was_idle)


@test("paged_meas", mutating=True)
def meas_rta_actions(dev, profile, chk):
    """RTA_START / STOP / RESET are zero-length SETs that move the engine as documented; payloads and GETs are refused."""
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
        chk.no_stall(lambda: _set(dev, "RTA_STOP"), "second RTA_STOP")
        chk.eq(_rta_state(dev), RTA_STATE_IDLE, "idle after the second RTA_STOP")
        chk.no_stall(lambda: _set(dev, "RTA_START"), "second RTA_START")
        chk.eq(_rta_state(dev) != RTA_STATE_IDLE, started != RTA_STATE_IDLE, "both starts reach the same state")
        if live and started == RTA_STATE_IDLE:
            chk.ok(False, f"RTA_START left the engine idle with live mask 0x{live:04X}")
        chk.no_stall(lambda: _set(dev, "RTA_RESET"), "RTA_RESET while running")
    finally:
        _rta_restore_run(dev, was_idle)


@test("paged_meas", mutating=True)
def meas_siggen_config(dev, profile, chk):
    """SIGGEN_CONFIG is 36 bytes in the caps' version and refuses bad configs; a valid idle config round-trips."""
    _meas(dev)
    cfg = _get(dev, "SIGGEN_CONFIG")
    chk.eq(len(cfg), 36, "SIGGEN_CONFIG length")
    chk.eq(cfg[0], _get(dev, "SIGGEN_CAPS")[0], "SIGGEN_CONFIG version == SIGGEN_CAPS version")
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
        chk.no_stall(lambda: _set(dev, "SIGGEN_CONFIG", bytes(a)), "first SET")
        chk.eq(_get(dev, "SIGGEN_CONFIG"), bytes(a), "GET after the first SET")
        chk.no_stall(lambda: _set(dev, "SIGGEN_CONFIG", bytes(b)), "second SET")
        chk.eq(_get(dev, "SIGGEN_CONFIG"), bytes(b), "GET after the second SET")
        chk.ok(_siggen_idle(dev), "writing a config does not start the generator")
    finally:
        _set(dev, "SIGGEN_CONFIG", cfg)
    chk.eq(_get(dev, "SIGGEN_CONFIG"), cfg, "config restored")


@test("paged_meas")
def meas_siggen_status_caps_types(dev, profile, chk):
    """SIGGEN_STATUS / CAPS / TYPE_DESC have their sizes and agree with each other; a bad type is VC_ERR_ADDRESS."""
    _meas(dev)
    caps = _get(dev, "SIGGEN_CAPS")
    chk.eq(len(caps), 8, "SIGGEN_CAPS length")
    no = profile.num_output_channels
    chk.eq(caps[2], no, "caps output channels")
    chk.eq(struct.unpack_from("<H", caps, 4)[0], (1 << no) - 1, "caps valid channel mask")
    chk.eq(caps[6:8], b"\0\0", "caps reserved")
    st = _get(dev, "SIGGEN_STATUS")
    chk.eq(len(st), 16, "SIGGEN_STATUS length")
    chk.eq(st[0], caps[0], "SIGGEN_STATUS version == SIGGEN_CAPS version")
    chk.ok(st[2] < caps[1], f"status signal type {st[2]} below the type count {caps[1]}")
    chk.eq(st[11], 0, "SIGGEN_STATUS reserved")
    count = caps[1]
    for i in range(count):
        d = _get(dev, "SIGGEN_TYPE_DESC", addr=i)
        if not chk.eq(len(d), 62, f"SIGGEN_TYPE_DESC {i} length"):
            continue
        chk.eq(d[0], i, f"SIGGEN_TYPE_DESC {i} echoes its id")
        name = d[1:9].split(b"\0")[0]
        chk.ok(name and all(0x20 <= b < 0x7F for b in name), f"type {i} name {d[1:9]!r}")
        chk.in_range(d[9], 0, 2, f"type {i} timing model")
        for k in range(4):
            sem, lo, hi, df = struct.unpack_from("<Bfff", d, 10 + 13 * k)
            if sem:
                chk.ok(all(math.isfinite(v) for v in (lo, hi, df)) and lo <= df <= hi,
                       f"type {i} param {k}: min {lo} <= def {df} <= max {hi}")
    for bad in (count, 0x100, 0xFFFF):
        _expect_stall(chk, lambda: _get(dev, "SIGGEN_TYPE_DESC", addr=bad), "VC_ERR_ADDRESS", f"type {bad}")
    _expect_stall(chk, lambda: _get(dev, "SIGGEN_CAPS", addr=0xFFFF), "VC_ERR_ADDRESS", "SIGGEN_CAPS at 0xFFFF")


@test("paged_meas", mutating=True)
def meas_siggen_actions_never_start(dev, profile, chk):
    """SIGGEN_START only meets refusals; STOP and STOP_NOW are harmless while idle, twice over."""
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
    for n in ("SIGGEN_STOP", "SIGGEN_STOP_NOW"):
        chk.no_stall(lambda: _set(dev, n), f"{n} again while idle")
    chk.ok(_siggen_idle(dev), "still idle after the repeated stop actions")
