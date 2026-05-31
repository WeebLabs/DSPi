"""
Identity, platform, status, and clip-clear commands.

Covers: 0x7E GET_SERIAL, 0x7F GET_PLATFORM, 0x50 GET_STATUS (+ sub-queries),
0x83 CLEAR_CLIPS.  All read-only / non-mutating except clear-clips.
"""

from ..device import OP
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
    """0x7F returns [platform, fw_major, fw_minor.patch BCD, num_output_channels]."""
    p = dev.get(OP.GET_PLATFORM, 4)
    chk.eq(len(p), 4, "platform length")
    chk.member(p[0], (0, 1), "platform id")
    chk.eq(p[3], profile.num_output_channels, "output-channel count byte")
    # BCD nibbles must be valid decimal digits.
    chk.in_range(p[1], 0, 99, "fw major byte")
    chk.in_range((p[2] >> 4) & 0xF, 0, 9, "fw minor nibble")
    chk.in_range(p[2] & 0xF, 0, 9, "fw patch nibble")


@test("identity")
def status_combined(dev, profile, chk):
    """0x50 wValue=9 returns peaks[NUM_CHANNELS] + cpu0 + cpu1 + clip_flags."""
    expected_len = profile.num_channels * 2 + 4
    data = dev.get(OP.GET_STATUS, 64, wvalue=9)
    chk.ok(len(data) >= expected_len, f"status len {len(data)} < {expected_len}")
    if len(data) < expected_len:
        return
    import struct
    peaks = struct.unpack_from(f"<{profile.num_channels}H", data, 0)
    cpu0 = data[profile.num_channels * 2]
    cpu1 = data[profile.num_channels * 2 + 1]
    clip = struct.unpack_from("<H", data, profile.num_channels * 2 + 2)[0]
    # Idle: peaks small (not strictly 0 — may retain residue), cpu plausible.
    for i, pk in enumerate(peaks):
        chk.in_range(pk, 0, 65535, f"peak[{i}] range")
    chk.in_range(cpu0, 0, 100, "cpu0 load")
    chk.in_range(cpu1, 0, 100, "cpu1 load")
    chk.in_range(clip, 0, 0xFFFF, "clip_flags range")
    chk.note(f"idle peaks max={max(peaks)} cpu0={cpu0} cpu1={cpu1} clip=0x{clip:04X}")


@test("identity")
def status_subqueries(dev, profile, chk):
    """0x50 diagnostic sub-queries return plausible values; unknown wValue -> zeros, no STALL."""
    fs = dev.get_u32(OP.GET_STATUS, wvalue=15)        # sample rate
    chk.member(fs, (44100, 48000, 88200, 96000), "sample rate")
    clk = dev.get_u32(OP.GET_STATUS, wvalue=13)       # clk_sys Hz
    chk.ok(clk > 1_000_000, f"clk_sys implausible: {clk}")
    mounted = dev.get_u32(OP.GET_STATUS, wvalue=12)
    chk.member(mounted, (0, 1), "mounted flag")
    # Unknown sub-query must return zeros, never STALL.
    unk = chk.no_stall(lambda: dev.get_u32(OP.GET_STATUS, wvalue=200), "unknown sub-query")
    chk.eq(unk, 0, "unknown sub-query returns 0")
    chk.note(f"Fs={fs} clk_sys={clk} mounted={mounted}")


@test("identity", mutating=True)
def clear_clips_read_then_clear(dev, profile, chk):
    """0x83 returns prior clip flags then zeroes them; second read must be 0 (idle)."""
    first = dev.get(OP.CLEAR_CLIPS, 2)
    chk.eq(len(first), 2, "clear-clips length")
    second = dev.get(OP.CLEAR_CLIPS, 2)
    val2 = second[0] | (second[1] << 8)
    chk.eq(val2, 0, "second clear-clips must be 0 when idle")
    # And the combined status clip field must also read 0 now.
    import struct
    st = dev.get(OP.GET_STATUS, profile.num_channels * 2 + 4, wvalue=9)
    clip = struct.unpack_from("<H", st, profile.num_channels * 2 + 2)[0]
    chk.eq(clip, 0, "status clip_flags 0 after clear")
