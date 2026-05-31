"""
Output slots / Core1 / pins / output-type / I2S / MCK group.

Output enable/gain/mute/delay 0x72-0x79
Core1 mode / conflict          0x7A/0x7B
Output pin                     0x7C/0x7D
Output type (SPDIF/I2S)        0xC0/0xC1
I2S BCK pin                    0xC2/0xC3
MCK enable/pin/multiplier      0xC4-0xC9

Every test here that mutates hardware (enables, pins, types, MCK) captures the
prior state and restores it in a finally block — pin/type moves run a muted
pipeline reset, so leaving them perturbed would violate the slot-alignment
constraint for later tests.
"""

import time

from ..device import OP, Stall
from ..framework import test

# PIN_CONFIG_* status codes.
PIN_SUCCESS, PIN_INVALID_PIN, PIN_IN_USE, PIN_INVALID_OUTPUT, PIN_OUTPUT_ACTIVE = range(5)
# Output type enum.
TYPE_SPDIF, TYPE_I2S = 0, 1
# Core1 mode enum.
CORE1_IDLE, CORE1_PDM, CORE1_EQ_WORKER = 0, 1, 2


def _enables(dev, n):
    return [dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=i) for i in range(n)]


def _set_enables(dev, states):
    # Disable conflicting ones first (PDM vs EQ-worker interlock), then enable.
    for i, v in enumerate(states):
        if not v:
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 0, wvalue=i)
    for i, v in enumerate(states):
        if v:
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=i)


def _busy_pins(dev, profile):
    pins = set(dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=i) for i in range(profile.num_pin_outputs))
    pins.add(dev.get_u8(OP.GET_SPDIF_RX_PIN))
    bck = dev.get_u8(OP.GET_I2S_BCK_PIN)
    pins.add(bck)
    pins.add(bck + 1)
    if dev.get_u8(OP.GET_MCK_ENABLE):
        pins.add(dev.get_u8(OP.GET_MCK_PIN))
    pins.update([12, 23, 24, 25])  # invalid/reserved
    return pins


def _free_pin(dev, profile, prefer=(16, 17, 18, 19, 20, 21, 22, 26, 27)):
    busy = _busy_pins(dev, profile)
    for p in prefer:
        if p not in busy and p <= 29:
            return p
    return None


# --- enable / gain / mute / delay -------------------------------------------

@test("outputs", mutating=True)
def output_enable_roundtrip(dev, profile, chk):
    """0x72/0x73 enable round-trips on the always-safe core-0 outputs (0,1)."""
    saved = _enables(dev, profile.num_output_channels)
    try:
        for i in (0, 1):
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=i)
            chk.eq(dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=i), 1, f"out{i} enable->1")
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 0, wvalue=i)
            chk.eq(dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=i), 0, f"out{i} enable->0")
        chk.stalls(lambda: dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=profile.num_output_channels),
                   "GET enable idx==count STALL")
        chk.no_stall(lambda: dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=profile.num_output_channels),
                     "SET enable bad idx no STALL")
    finally:
        _set_enables(dev, saved)


@test("outputs", mutating=True)
def output_gain_roundtrip(dev, profile, chk):
    """0x74/0x75 gain (float dB, no clamp); GET idx>=count STALLs."""
    g0 = dev.get_f32(OP.GET_OUTPUT_GAIN, wvalue=0)
    try:
        dev.set_f32(OP.SET_OUTPUT_GAIN, -6.0, wvalue=0)
        chk.approx(dev.get_f32(OP.GET_OUTPUT_GAIN, wvalue=0), -6.0, 1e-3, "gain roundtrip")
        dev.set_f32(OP.SET_OUTPUT_GAIN, 12.0, wvalue=0)
        chk.approx(dev.get_f32(OP.GET_OUTPUT_GAIN, wvalue=0), 12.0, 1e-3, "gain +12 (no clamp)")
        chk.stalls(lambda: dev.get_f32(OP.GET_OUTPUT_GAIN, wvalue=profile.num_output_channels),
                   "GET gain bad idx STALL")
    finally:
        dev.set_f32(OP.SET_OUTPUT_GAIN, g0, wvalue=0)


@test("outputs", mutating=True)
def output_mute_raw(dev, profile, chk):
    """0x76/0x77 mute stored RAW (not normalized); GET idx>=count STALLs."""
    m0 = dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=0)
    try:
        dev.set_u8(OP.SET_OUTPUT_MUTE, 1, wvalue=0)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=0), 1, "mute 1")
        dev.set_u8(OP.SET_OUTPUT_MUTE, 0, wvalue=0)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=0), 0, "mute 0")
        dev.set_u8(OP.SET_OUTPUT_MUTE, 0x7F, wvalue=0)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=0), 0x7F, "mute stored raw (0x7F)")
        chk.stalls(lambda: dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=profile.num_output_channels),
                   "GET mute bad idx STALL")
    finally:
        dev.set_u8(OP.SET_OUTPUT_MUTE, m0, wvalue=0)


@test("outputs", mutating=True)
def output_delay_clamp(dev, profile, chk):
    """0x78/0x79 output delay: negative clamps to 0; GET idx>=count STALLs."""
    d0 = dev.get_f32(OP.GET_OUTPUT_DELAY, wvalue=0)
    try:
        dev.set_f32(OP.SET_OUTPUT_DELAY, 5.0, wvalue=0)
        chk.approx(dev.get_f32(OP.GET_OUTPUT_DELAY, wvalue=0), 5.0, 1e-3, "delay roundtrip")
        dev.set_f32(OP.SET_OUTPUT_DELAY, -3.0, wvalue=0)
        chk.approx(dev.get_f32(OP.GET_OUTPUT_DELAY, wvalue=0), 0.0, 1e-3, "negative delay -> 0")
        chk.stalls(lambda: dev.get_f32(OP.GET_OUTPUT_DELAY, wvalue=profile.num_output_channels),
                   "GET delay bad idx STALL")
    finally:
        dev.set_f32(OP.SET_OUTPUT_DELAY, d0, wvalue=0)


# --- Core1 mode / conflict --------------------------------------------------

@test("outputs", mutating=True)
def core1_mode_and_conflict_interlock(dev, profile, chk):
    """0x7A/0x7B + 0x72 interlock: PDM(last) and EQ-worker(2..n-2) are mutually exclusive."""
    if profile.num_output_channels < 9:
        # RP2040 has a smaller output map; still exercise but ranges differ.
        pass
    pdm = profile.num_output_channels - 1   # PDM = last output (8 on RP2350)
    eq_out = 2                              # first Core1 EQ-worker output
    saved = _enables(dev, profile.num_output_channels)
    try:
        # Clear everything Core1-related.
        for i in range(2, profile.num_output_channels):
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 0, wvalue=i)
        chk.eq(dev.get_u8(OP.GET_CORE1_MODE), CORE1_IDLE, "mode IDLE when only 0/1 active")

        # Enable an EQ-worker output -> mode EQ_WORKER.
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=eq_out)
        chk.eq(dev.get_u8(OP.GET_CORE1_MODE), CORE1_EQ_WORKER, "mode EQ_WORKER after enabling out2")
        # Conflict predicate says enabling PDM would conflict.
        chk.eq(dev.get_u8(OP.GET_CORE1_CONFLICT, wvalue=pdm), 1, "0x7B predicts PDM conflict")
        chk.eq(dev.get_u8(OP.GET_CORE1_CONFLICT, wvalue=3), 0, "0x7B no conflict between EQ outputs")
        # The interlock actually drops the PDM enable.
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=pdm)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=pdm), 0, "PDM enable dropped by interlock")

        # Reverse: clear EQ outputs, enable PDM -> mode PDM.
        for i in range(2, profile.num_output_channels):
            dev.set_u8(OP.SET_OUTPUT_ENABLE, 0, wvalue=i)
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=pdm)
        chk.eq(dev.get_u8(OP.GET_CORE1_MODE), CORE1_PDM, "mode PDM after enabling PDM")
        chk.eq(dev.get_u8(OP.GET_CORE1_CONFLICT, wvalue=eq_out), 1, "0x7B predicts EQ conflict while PDM on")
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=eq_out)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_ENABLE, wvalue=eq_out), 0, "EQ enable dropped by interlock")
        # Out-of-range conflict query returns 0 (not STALL).
        chk.eq(chk.no_stall(lambda: dev.get_u8(OP.GET_CORE1_CONFLICT, wvalue=profile.num_output_channels),
                            "conflict bad idx no STALL"), 0, "conflict bad idx -> 0")
    finally:
        _set_enables(dev, saved)


# --- Output pins ------------------------------------------------------------

@test("outputs", mutating=True)
def output_pin_status_codes(dev, profile, chk):
    """0x7C status codes: INVALID_OUTPUT / INVALID_PIN / PIN_IN_USE without changing state."""
    # Bad output index -> INVALID_OUTPUT.
    st = dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(0 << 8) | profile.num_pin_outputs)
    chk.eq(st, PIN_INVALID_OUTPUT, "bad output idx -> INVALID_OUTPUT")
    # Reserved pin 12 -> INVALID_PIN.
    st = dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(12 << 8) | 0)
    chk.eq(st, PIN_INVALID_PIN, "pin 12 -> INVALID_PIN")
    # A pin already used by another output -> PIN_IN_USE.
    other = dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=1)
    st = dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(other << 8) | 0)
    chk.eq(st, PIN_IN_USE, f"pin {other} in use -> PIN_IN_USE")
    # 0x7D GET out of range STALLs.
    chk.stalls(lambda: dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=profile.num_pin_outputs), "GET pin bad idx STALL")


@test("outputs", mutating=True)
def output_pin_move_roundtrip(dev, profile, chk):
    """0x7C/0x7D: move a SPDIF output to a free pin (SUCCESS) and restore (pipeline reset)."""
    free = _free_pin(dev, profile)
    if free is None:
        chk.note("no free GPIO available to test pin move; skipping move")
        return
    orig = dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0)
    try:
        st = dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(free << 8) | 0)
        chk.eq(st, PIN_SUCCESS, f"move out0 -> pin {free} SUCCESS")
        time.sleep(0.1)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0), free, "pin reflects move")
        # No-op: setting same pin again returns SUCCESS.
        chk.eq(dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(free << 8) | 0), PIN_SUCCESS, "same-pin no-op SUCCESS")
    finally:
        dev.get_u8(OP.SET_OUTPUT_PIN, wvalue=(orig << 8) | 0)
        time.sleep(0.1)
        chk.eq(dev.get_u8(OP.GET_OUTPUT_PIN, wvalue=0), orig, "pin restored")


# --- Output type (SPDIF <-> I2S) --------------------------------------------

def _poll_type(dev, slot, want, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=slot) == want:
            return True
        time.sleep(0.03)
    return False


@test("outputs", mutating=True)
def output_type_validation(dev, profile, chk):
    """0xC0 status codes: bad slot -> INVALID_OUTPUT, bad type -> INVALID_PIN; GET bad slot STALL."""
    st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | profile.num_spdif)
    chk.eq(st, PIN_INVALID_OUTPUT, "bad slot -> INVALID_OUTPUT")
    st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(2 << 8) | 0)
    chk.eq(st, PIN_INVALID_PIN, "type 2 -> INVALID_PIN")
    chk.stalls(lambda: dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=profile.num_spdif), "GET type bad slot STALL")


@test("outputs", mutating=True)
def output_type_switch_roundtrip(dev, profile, chk):
    """0xC0/0xC1: slot 0 SPDIF->I2S->SPDIF; deferred switch applies; device stays responsive."""
    orig = dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=0)
    target = TYPE_I2S if orig == TYPE_SPDIF else TYPE_SPDIF
    try:
        st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(target << 8) | 0)
        chk.eq(st, PIN_SUCCESS, f"switch slot0 -> {target} accepted")
        chk.ok(_poll_type(dev, 0, target), f"slot0 type became {target} (deferred)")
        # Liveness: control plane still answers (proves USBCTRL IRQ was re-enabled).
        chk.ok(dev.is_alive(), "device responsive after type switch")
    finally:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(orig << 8) | 0)
        chk.ok(_poll_type(dev, 0, orig), "slot0 type restored")


# --- I2S BCK / MCK ----------------------------------------------------------

@test("outputs", mutating=True)
def i2s_bck_pin(dev, profile, chk):
    """0xC2/0xC3 BCK pin: invalid pin -> INVALID_PIN; valid free move round-trips (when no I2S active)."""
    orig = dev.get_u8(OP.GET_I2S_BCK_PIN)
    # 11 is invalid because 11+1=12 (UART TX) fails the pair validity check.
    chk.eq(dev.get_u8(OP.SET_I2S_BCK_PIN, wvalue=11), PIN_INVALID_PIN, "BCK pin 11 (pair hits 12) -> INVALID_PIN")
    any_i2s = any(dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=s) == TYPE_I2S for s in range(profile.num_spdif))
    if any_i2s:
        chk.note("an I2S slot is active; BCK move correctly returns OUTPUT_ACTIVE — skipping move")
        chk.eq(dev.get_u8(OP.SET_I2S_BCK_PIN, wvalue=16), PIN_OUTPUT_ACTIVE, "BCK move blocked while I2S active")
        return
    free = _free_pin(dev, profile, prefer=(16, 18, 20, 26))
    if free is None or (free + 1) in _busy_pins(dev, profile) or free + 1 > 29:
        chk.note("no free BCK pin pair; skipping move")
        return
    try:
        chk.eq(dev.get_u8(OP.SET_I2S_BCK_PIN, wvalue=free), PIN_SUCCESS, f"BCK move -> {free}")
        chk.eq(dev.get_u8(OP.GET_I2S_BCK_PIN), free, "BCK pin reflects move")
    finally:
        dev.get_u8(OP.SET_I2S_BCK_PIN, wvalue=orig)
        chk.eq(dev.get_u8(OP.GET_I2S_BCK_PIN), orig, "BCK pin restored")


@test("outputs", mutating=True)
def mck_enable_and_multiplier(dev, profile, chk):
    """0xC4/0xC5 enable and 0xC8/0xC9 multiplier (wire-encoded 0=128x, 1=256x; >1 -> INVALID_PIN)."""
    en0 = dev.get_u8(OP.GET_MCK_ENABLE)
    mult0 = dev.get_u8(OP.GET_MCK_MULTIPLIER)
    try:
        # Multiplier encoding.
        chk.eq(dev.get_u8(OP.SET_MCK_MULTIPLIER, wvalue=0), PIN_SUCCESS, "mult 0 (128x) accepted")
        chk.eq(dev.get_u8(OP.GET_MCK_MULTIPLIER), 0, "mult reads 0 (128x)")
        chk.eq(dev.get_u8(OP.SET_MCK_MULTIPLIER, wvalue=1), PIN_SUCCESS, "mult 1 (256x) accepted")
        chk.eq(dev.get_u8(OP.GET_MCK_MULTIPLIER), 1, "mult reads 1 (256x)")
        chk.eq(dev.get_u8(OP.SET_MCK_MULTIPLIER, wvalue=2), PIN_INVALID_PIN, "mult 2 -> INVALID_PIN")
        chk.eq(dev.get_u8(OP.GET_MCK_MULTIPLIER), 1, "mult unchanged after bad set")
        # Enable round-trip.
        chk.eq(dev.get_u8(OP.SET_MCK_ENABLE, wvalue=1), PIN_SUCCESS, "MCK enable accepted")
        chk.eq(dev.get_u8(OP.GET_MCK_ENABLE), 1, "MCK enabled")
        # While enabled, moving the MCK pin must be refused.
        chk.eq(dev.get_u8(OP.SET_MCK_PIN, wvalue=21), PIN_OUTPUT_ACTIVE, "MCK pin move blocked while enabled")
        chk.eq(dev.get_u8(OP.SET_MCK_ENABLE, wvalue=0), PIN_SUCCESS, "MCK disable accepted")
        chk.eq(dev.get_u8(OP.GET_MCK_ENABLE), 0, "MCK disabled")
    finally:
        # SET_MCK_* are SET-via-wValue (IN dir, return a status byte) -> use get_u8.
        dev.get_u8(OP.SET_MCK_MULTIPLIER, wvalue=mult0)
        if en0:
            dev.get_u8(OP.SET_MCK_ENABLE, wvalue=1)


@test("outputs", mutating=True)
def mck_pin_validation(dev, profile, chk):
    """0xC6/0xC7 MCK pin must map to a hardware CLK_GPOUT; non-GPOUT/reserved -> INVALID_PIN."""
    en0 = dev.get_u8(OP.GET_MCK_ENABLE)
    if en0:
        dev.get_u8(OP.SET_MCK_ENABLE, wvalue=0)
    orig = dev.get_u8(OP.GET_MCK_PIN)
    try:
        # Pin 20 is a valid GPIO but not a GPOUT-capable pin -> INVALID_PIN.
        chk.eq(dev.get_u8(OP.SET_MCK_PIN, wvalue=20), PIN_INVALID_PIN, "non-GPOUT pin 20 -> INVALID_PIN")
        chk.eq(dev.get_u8(OP.SET_MCK_PIN, wvalue=12), PIN_INVALID_PIN, "reserved pin 12 -> INVALID_PIN")
        # A GPOUT-capable, free pin should succeed (21 on both platforms).
        if 21 not in _busy_pins(dev, profile):
            chk.eq(dev.get_u8(OP.SET_MCK_PIN, wvalue=21), PIN_SUCCESS, "GPOUT pin 21 SUCCESS")
            chk.eq(dev.get_u8(OP.GET_MCK_PIN), 21, "MCK pin reflects 21")
    finally:
        dev.get_u8(OP.SET_MCK_PIN, wvalue=orig)
        if en0:
            dev.get_u8(OP.SET_MCK_ENABLE, wvalue=1)
