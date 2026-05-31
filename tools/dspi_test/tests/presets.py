"""
Presets / channel names / bulk-params group.

Non-flash: invalid-slot in-band errors, name/dir/startup/active reads, channel
name round-trip (RAM), bulk GET/SET round-trip + version/size guards.

Flash-gated (--allow-flash): preset save/load/delete lifecycle, preset name,
startup, include-pins — all bracketed and restored, using one scratch slot.
"""

import struct
import time

from ..device import OP, Stall
from ..framework import test

PRESET_SLOTS = 10
PRESET_OK = 0x00
PRESET_ERR_INVALID_SLOT = 0x01


def _dir(dev):
    d = dev.get(OP.PRESET_GET_DIR, 7)
    return {
        "occupied": d[0] | (d[1] << 8),
        "startup_mode": d[2], "default_slot": d[3], "active": d[4],
        "include_pins": d[5], "mv_mode": d[6],
    }


def _scratch_slot(dev):
    occ = _dir(dev)["occupied"]
    for s in range(PRESET_SLOTS - 1, -1, -1):
        if not (occ & (1 << s)):
            return s
    return None


def _poll(fn, want, timeout_s=1.5):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if fn() == want:
            return True
        time.sleep(0.04)
    return False


# --- Non-flash --------------------------------------------------------------

@test("presets")
def preset_invalid_slot_inband(dev, profile, chk):
    """Save/load/delete with slot >= PRESET_SLOTS return PRESET_ERR_INVALID_SLOT in-band (no STALL, no flash)."""
    for op, name in [(OP.PRESET_SAVE, "save"), (OP.PRESET_LOAD, "load"), (OP.PRESET_DELETE, "delete")]:
        r = chk.no_stall(lambda op=op: dev.get(op, 1, wvalue=PRESET_SLOTS), f"{name} slot=10 no STALL")
        if r is not None:
            chk.eq(r[0], PRESET_ERR_INVALID_SLOT, f"{name} slot=10 -> INVALID_SLOT")
        r = chk.no_stall(lambda op=op: dev.get(op, 1, wvalue=0xFF), f"{name} slot=255 no STALL")
        if r is not None:
            chk.eq(r[0], PRESET_ERR_INVALID_SLOT, f"{name} slot=255 -> INVALID_SLOT")


@test("presets")
def preset_name_and_dir_reads(dev, profile, chk):
    """0x93 name (STALL slot>=10), 0x95 dir (7B), 0x97 startup (3B), 0x99 include-pins, 0x9A active."""
    chk.eq(len(dev.get(OP.PRESET_GET_NAME, 32, wvalue=0)), 32, "preset 0 name 32B")
    chk.stalls(lambda: dev.get(OP.PRESET_GET_NAME, 32, wvalue=PRESET_SLOTS), "name slot=10 STALL")
    d = _dir(dev)
    chk.member(d["startup_mode"], (0, 1), "startup mode")
    chk.in_range(d["default_slot"], 0, PRESET_SLOTS - 1, "default slot")
    chk.in_range(d["active"], 0, PRESET_SLOTS - 1, "active slot")
    chk.member(d["include_pins"], (0, 1), "include_pins")
    chk.member(d["mv_mode"], (0, 1), "mv mode")
    chk.eq(len(dev.get(OP.PRESET_GET_STARTUP, 3)), 3, "startup 3B")
    chk.member(dev.get_u8(OP.PRESET_GET_INCLUDE_PINS), (0, 1), "include-pins byte")
    chk.in_range(dev.get_u8(OP.PRESET_GET_ACTIVE), 0, PRESET_SLOTS - 1, "active byte")


@test("presets", mutating=True)
def channel_name_roundtrip(dev, profile, chk):
    """0x9B/0x9C channel names round-trip (RAM, no flash); truncate to 31 chars; ch>=count STALLs."""
    ch = 2
    orig = dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=ch)
    try:
        dev.set(OP.SET_CHANNEL_NAME, b"QA Test Name", wvalue=ch)
        got = dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=ch).split(b"\x00", 1)[0]
        chk.eq(got, b"QA Test Name", "channel name round-trip")
        # 40-char payload truncates to 31 + NUL.
        dev.set(OP.SET_CHANNEL_NAME, b"A" * 40, wvalue=ch)
        got = dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=ch).split(b"\x00", 1)[0]
        chk.eq(len(got), 31, "name truncated to 31 chars")
        chk.stalls(lambda: dev.get(OP.GET_CHANNEL_NAME, 32, wvalue=profile.num_channels), "ch>=count STALL")
        chk.no_stall(lambda: dev.set(OP.SET_CHANNEL_NAME, b"x", wvalue=profile.num_channels),
                     "SET bad ch no STALL")
    finally:
        dev.set(OP.SET_CHANNEL_NAME, orig.split(b"\x00", 1)[0], wvalue=ch)


@test("presets")
def bulk_get_header(dev, profile, chk):
    """0xA0 header reports the wire format version, platform, and channel geometry."""
    hdr = dev.get(OP.GET_ALL_PARAMS, 16)
    (fmt, plat, nch, nout, nin, mbands, plen) = struct.unpack_from("<BBBBBBH", hdr, 0)
    chk.eq(fmt, profile.wire_format_version, "format version")
    chk.eq(plat, profile.platform_id, "platform id")
    chk.eq(nch, profile.num_channels, "num_channels")
    chk.eq(nout, profile.num_output_channels, "num_output_channels")
    chk.eq(nin, profile.num_input_channels, "num_input_channels")
    chk.eq(plen, profile.bulk_payload_len, "payload length")


@test("presets", mutating=True)
def bulk_roundtrip_apply(dev, profile, chk):
    """0xA1 apply is idempotent (reaches a stable fixpoint) and live edits show up in 0xA0 (0 flash).

    Note: a single GET->SET->GET is NOT byte-identical for arbitrary state because
    0xA1 normalizes some fields (e.g. out-of-range EQ freq/Q are clamped on apply,
    unlike the raw-storing 0x42 path).  The correct invariant is a fixpoint:
    applying a blob, then applying the result, yields the same bytes.
    """
    # Two applies reach a fixpoint: the first normalizes (e.g. clamps out-of-range
    # EQ freq/Q), the second reproduces the normalized blob exactly.  A
    # re-enumeration mid-apply reloads the active preset and breaks the compare,
    # so only assert on a re-enum-free attempt; surface the re-enum otherwise.
    ok = False
    for attempt in range(3):
        a = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
        dev.set(OP.SET_ALL_PARAMS, a)
        dev.wait_ready()
        time.sleep(0.1)
        c = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
        reenum0 = dev.reenumerations
        dev.set(OP.SET_ALL_PARAMS, c)
        dev.wait_ready()
        time.sleep(0.1)
        d = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
        if dev.reenumerations != reenum0:
            chk.note(f"re-enumeration during bulk apply (attempt {attempt}); device reloaded preset — retrying")
            continue
        chk.eq(c, d, "bulk apply reaches a stable fixpoint (idempotent round-trip)")
        ok = True
        break
    if not ok:
        chk.note("could not complete a re-enum-free bulk fixpoint in 3 tries (device re-enumerates under bulk apply)")
        chk.ok(dev.is_alive(), "device still responsive after bulk-apply re-enum churn")
    # A live param change is reflected in the next bulk snapshot.
    byp = dev.get_u8(OP.GET_BYPASS)
    try:
        dev.set_u8(OP.SET_BYPASS, 0 if byp else 1)
        e = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
        chk.ne(e, d, "bulk snapshot reflects a live param change")
    finally:
        dev.set_u8(OP.SET_BYPASS, byp)


@test("presets", mutating=True)
def bulk_version_and_size_guards(dev, profile, chk):
    """0xA1 silently ignores a bad header version (no STALL, no change); oversized/short-large STALL."""
    a = bytearray(dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len))
    bad = bytearray(a)
    bad[0] = 99  # format_version out of range
    chk.no_stall(lambda: dev.set(OP.SET_ALL_PARAMS, bytes(bad)), "bad-version SET no STALL")
    dev.wait_ready()
    time.sleep(0.1)
    after = dev.get_ready(OP.GET_ALL_PARAMS, profile.bulk_payload_len)
    chk.eq(after, bytes(a), "bad-version bulk SET ignored (state unchanged)")
    # Oversized and short-but-large both STALL at the transport/gate.
    chk.stalls(lambda: dev.set(OP.SET_ALL_PARAMS, b"\x00" * (profile.bulk_payload_len + 1)), "oversized bulk STALL")
    chk.stalls(lambda: dev.set(OP.SET_ALL_PARAMS, b"\x00" * 100), "short-but->64 bulk STALL")


# --- Flash-gated ------------------------------------------------------------

@test("presets", mutating=True, flash=6)
def preset_save_load_delete_lifecycle(dev, profile, chk):
    """Full preset lifecycle on a scratch slot: save->load->delete, deferred completion polled, restored."""
    scratch = _scratch_slot(dev)
    if scratch is None:
        chk.note("all preset slots occupied — skipping lifecycle to avoid clobbering user data")
        return
    orig_active = dev.get_u8(OP.PRESET_GET_ACTIVE)
    try:
        # Save current live state to the scratch slot.
        r = dev.get(OP.PRESET_SAVE, 1, wvalue=scratch)
        chk.eq(r[0], PRESET_OK, "save accepted")
        chk.ok(_poll(lambda: dev.get_u8(OP.PRESET_GET_ACTIVE), scratch), "active became scratch after save")
        chk.ok((_dir(dev)["occupied"] >> scratch) & 1, "scratch slot now occupied")
        # Load it back.
        r = dev.get(OP.PRESET_LOAD, 1, wvalue=scratch)
        chk.eq(r[0], PRESET_OK, "load accepted")
        dev.wait_ready()
        chk.ok(_poll(lambda: dev.get_u8(OP.PRESET_GET_ACTIVE), scratch), "active is scratch after load")
        chk.ok(dev.is_alive(), "device responsive after load")
        # Delete it.
        r = dev.get(OP.PRESET_DELETE, 1, wvalue=scratch)
        chk.eq(r[0], PRESET_OK, "delete accepted")
        chk.ok(_poll(lambda: (_dir(dev)["occupied"] >> scratch) & 1, 0), "scratch slot freed after delete")
    finally:
        # Restore the original active slot pointer (global restore fixes live state after).
        dev.get(OP.PRESET_LOAD, 1, wvalue=orig_active)
        dev.wait_ready()


@test("presets", mutating=True, flash=2)
def preset_name_roundtrip(dev, profile, chk):
    """0x94/0x93 preset name round-trips through flash on a scratch slot and is restored."""
    scratch = _scratch_slot(dev)
    if scratch is None:
        chk.note("no scratch slot; skipping preset name test")
        return
    orig = dev.get(OP.PRESET_GET_NAME, 32, wvalue=scratch).split(b"\x00", 1)[0]
    try:
        dev.set(OP.PRESET_SET_NAME, b"QA Scratch", wvalue=scratch)
        chk.ok(_poll(lambda: dev.get(OP.PRESET_GET_NAME, 32, wvalue=scratch).split(b"\x00", 1)[0],
                     b"QA Scratch"), "preset name persisted")
    finally:
        dev.set(OP.PRESET_SET_NAME, orig, wvalue=scratch)
        dev.wait_ready()


@test("presets", mutating=True, flash=2)
def preset_startup_roundtrip(dev, profile, chk):
    """0x96/0x97 startup config round-trips through flash and is restored."""
    s = dev.get(OP.PRESET_GET_STARTUP, 3)
    orig_mode, orig_slot = s[0], s[1]
    new_mode = 0 if orig_mode == 1 else 1
    try:
        dev.set(OP.PRESET_SET_STARTUP, bytes([new_mode, orig_slot]))
        chk.ok(_poll(lambda: dev.get(OP.PRESET_GET_STARTUP, 3)[0], new_mode), "startup mode persisted")
        # Invalid mode (2) is a silent no-op.
        dev.set(OP.PRESET_SET_STARTUP, bytes([2, orig_slot]))
        time.sleep(0.1)
        chk.eq(dev.get(OP.PRESET_GET_STARTUP, 3)[0], new_mode, "invalid mode ignored")
    finally:
        dev.set(OP.PRESET_SET_STARTUP, bytes([orig_mode, orig_slot]))
        dev.wait_ready()


@test("presets", mutating=True, flash=2)
def preset_include_pins_roundtrip(dev, profile, chk):
    """0x98/0x99 include-pins flag round-trips through flash and is restored."""
    orig = dev.get_u8(OP.PRESET_GET_INCLUDE_PINS)
    try:
        dev.set_u8(OP.PRESET_SET_INCLUDE_PINS, 0 if orig else 1)
        chk.ok(_poll(lambda: dev.get_u8(OP.PRESET_GET_INCLUDE_PINS), 0 if orig else 1),
               "include-pins toggled")
    finally:
        dev.set_u8(OP.PRESET_SET_INCLUDE_PINS, orig)
        dev.wait_ready()
