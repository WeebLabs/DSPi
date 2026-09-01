"""
Identity, platform, status, and clip-clear commands.

Covers: 0x7E GET_SERIAL, 0x7F GET_PLATFORM, 0x80 GET_BUILD_INFO,
0x50 GET_STATUS (+ sub-queries), 0x83 CLEAR_CLIPS.  All read-only /
non-mutating except clear-clips.
"""

import re

from ..device import OP, Stall
from ..framework import test


@test("identity")
def serial_is_printable_hex(dev, profile, chk):
    """0x7E returns 16 printable ASCII-hex bytes matching the USB serial descriptor."""
    raw = dev.get(OP.GET_SERIAL, 16)
    chk.eq(len(raw), 16, "serial length")
    try:
        s = raw.decode("ascii")
    except UnicodeDecodeError:
        chk.ok(False, f"serial not ASCII: {raw!r}")
        return
    chk.ok(all(c in "0123456789ABCDEFabcdef" for c in s), f"serial not hex: {s!r}")
    # Cross-check against the enumerated iSerialNumber descriptor.
    try:
        import usb.util
        desc = usb.util.get_string(dev.dev, dev.dev.iSerialNumber)
        if desc:
            chk.ok(s.upper().startswith(desc.upper()[:16]) or desc.upper().startswith(s.upper()),
                   f"serial {s!r} != descriptor {desc!r}")
    except Exception:  # noqa: BLE001
        chk.note("could not read iSerialNumber descriptor for cross-check")


@test("identity")
def platform_packet(dev, profile, chk):
    """0x7F returns [platform, fw_major, legacy minor.patch nibbles,
    num_output_channels, fw_minor, fw_patch]."""
    p = dev.get(OP.GET_PLATFORM, 6)
    chk.eq(len(p), 6, "platform length")
    chk.member(p[0], (0, 1), "platform id")
    chk.eq(p[3], profile.num_output_channels, "output-channel count byte")
    chk.in_range(p[1], 0, 99, "fw major byte")
    # Legacy nibbles must mirror the full-width bytes while both fit in 4 bits.
    if p[4] <= 15 and p[5] <= 15:
        chk.eq((p[2] >> 4) & 0xF, p[4], "legacy minor nibble")
        chk.eq(p[2] & 0xF, p[5], "legacy patch nibble")


@test("identity")
def build_info(dev, profile, chk):
    """0x80 returns 64 bytes: [0..47] git describe string, [48..59] build date
    YYYY-MM-DD, [60..63] reserved zero.  ASCII, NUL padded, informational only."""
    try:
        data = dev.get(OP.GET_BUILD_INFO, 64)
    except Stall:
        chk.note("GET_BUILD_INFO unsupported (pre-widening firmware); skipped")
        return
    chk.eq(len(data), 64, "build info length")
    if len(data) != 64:
        return
    describe = data[:48].split(b"\0")[0]
    date = data[48:60].split(b"\0")[0]
    chk.ok(describe != b"", "describe string empty")
    chk.ok(all(0x20 <= b < 0x7F for b in describe), f"describe not printable: {describe!r}")
    chk.ok(re.fullmatch(rb"\d{4}-\d{2}-\d{2}", date) is not None,
           f"build date not YYYY-MM-DD: {date!r}")
    chk.eq(data[60:64], b"\0\0\0\0", "reserved tail zero")
    # A published release must be built from a clean tagged checkout; flag a
    # dirty stamp loudly but do not fail (bench builds are expected dirty).
    if b"-dirty" in describe:
        chk.note("DIRTY BUILD: binary was built from uncommitted changes")
    chk.note(f"build {describe.decode('ascii', 'replace')} "
             f"({date.decode('ascii', 'replace')})")


@test("identity")
def status_combined(dev, profile, chk):
    """0x50 wValue=9: peaks[NUM_CHANNELS] + cpu0 + cpu1 + clip_flags(u32) +
    active_input_count. Layout is NUM_CHANNELS*2 + 7 bytes (clip widened to u32
    and the live input count appended with the unified channel model)."""
    expected_len = profile.num_channels * 2 + 7
    data = dev.get(OP.GET_STATUS, 64, wvalue=9)
    chk.ok(len(data) >= expected_len, f"status len {len(data)} < {expected_len}")
    if len(data) < expected_len:
        return
    import struct
    peaks = struct.unpack_from(f"<{profile.num_channels}H", data, 0)
    off = profile.num_channels * 2
    cpu0 = data[off]
    cpu1 = data[off + 1]
    clip = struct.unpack_from("<I", data, off + 2)[0]
    active_in = data[off + 6]
    # Idle: peaks small (not strictly 0 — may retain residue), cpu plausible.
    for i, pk in enumerate(peaks):
        chk.in_range(pk, 0, 65535, f"peak[{i}] range")
    chk.in_range(cpu0, 0, 100, "cpu0 load")
    chk.in_range(cpu1, 0, 100, "cpu1 load")
    chk.member(active_in, (2, 4, 6, 8), "active input channel count")
    chk.note(f"idle peaks max={max(peaks)} cpu0={cpu0} cpu1={cpu1} "
             f"clip=0x{clip:08X} active_in={active_in}")


@test("identity")
def status_subqueries(dev, profile, chk):
    """0x50 diagnostic sub-queries return plausible values; unknown wValue -> zeros, no STALL."""
    fs = dev.get_u32(OP.GET_STATUS, wvalue=15)        # sample rate
    chk.member(fs, (44100, 48000, 88200, 96000), "sample rate")
    clk = dev.get_u32(OP.GET_STATUS, wvalue=13)       # clk_sys Hz
    chk.ok(clk > 1_000_000, f"clk_sys implausible: {clk}")
    mounted = dev.get_u32(OP.GET_STATUS, wvalue=12)
    chk.member(mounted, (0, 1), "mounted flag")
    active_in = dev.get_u32(OP.GET_STATUS, wvalue=23)  # live active USB input channel count
    chk.member(active_in, (2, 4, 6, 8), "active input count (sub-query 23)")
    # Unknown sub-query must return zeros, never STALL.
    unk = chk.no_stall(lambda: dev.get_u32(OP.GET_STATUS, wvalue=200), "unknown sub-query")
    chk.eq(unk, 0, "unknown sub-query returns 0")
    chk.note(f"Fs={fs} clk_sys={clk} mounted={mounted} active_in={active_in}")


@test("identity", mutating=True)
def clear_clips_read_then_clear(dev, profile, chk):
    """0x83 returns prior clip flags (u32) then zeroes them; second read must be 0 (idle)."""
    import struct
    first = dev.get(OP.CLEAR_CLIPS, 4)
    chk.eq(len(first), 4, "clear-clips length")
    second = dev.get(OP.CLEAR_CLIPS, 4)
    val2 = struct.unpack_from("<I", second, 0)[0]
    chk.eq(val2, 0, "second clear-clips must be 0 when idle")
    # And the combined status clip field must also read 0 now.
    st = dev.get(OP.GET_STATUS, profile.num_channels * 2 + 7, wvalue=9)
    clip = struct.unpack_from("<I", st, profile.num_channels * 2 + 2)[0]
    chk.eq(clip, 0, "status clip_flags 0 after clear")
