"""
Output enables and the core 1 interlock, output pins and slot types, the I2S BCK pair, MCK and output latency
(ROUTE.OUTPUT_ENABLE, SYS.CORE1_MODE, OUTPUT.PIN/TYPE/LATENCY*, CLOCK.I2S_BCK_PIN/MCK_*). Every hardware
change is restored in a finally block: pin and type moves restart all slots together, so a perturbed one
would break slot alignment for later tests.
"""

import struct
import time

from ..framework import test
from ..helpers import (TYPE_I2S, TYPE_SPDIF, busy_gpios, expect_result, expect_stall, free_gpio, gpio_owners,
                       held_by_others, invalid_gpios, poll_type, sample_rate, set_type, settle, wait_for)
from ..profile import MAX_GPIO

CORE1_IDLE, CORE1_PDM, CORE1_EQ_WORKER = 0, 1, 2


def _enables(dev, n):
    return [dev.pget_u8("ROUTE.OUTPUT_ENABLE", i) for i in range(n)]


def _set_enables(dev, states):
    # Disable first: an enable that meets the other core 1 mode is VC_ERR_STATE.
    for i, v in enumerate(states):
        if not v:
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 0, i)
    for i, v in enumerate(states):
        if v:
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, i)


# --- enable / core 1 --------------------------------------------------------

@test("outputs", mutating=True)
def output_enable_roundtrip(dev, profile, chk):
    """ROUTE.OUTPUT_ENABLE round-trips on the always-safe core 0 outputs (0, 1); past the end is VC_ERR_ADDRESS."""
    n = profile.num_output_channels
    saved = _enables(dev, n)
    try:
        for i in (0, 1):
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, i)
            chk.eq(dev.pget_u8("ROUTE.OUTPUT_ENABLE", i), 1, f"out{i} enable->1")
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 0, i)
            chk.eq(dev.pget_u8("ROUTE.OUTPUT_ENABLE", i), 0, f"out{i} enable->0")
        expect_stall(chk, lambda: dev.pget("ROUTE.OUTPUT_ENABLE", n, 1), "VC_ERR_ADDRESS", "GET enable idx==count")
        expect_stall(chk, lambda: dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, n), "VC_ERR_ADDRESS", "SET enable idx==count")
    finally:
        _set_enables(dev, saved)


@test("outputs", mutating=True)
def core1_mode_and_conflict_interlock(dev, profile, chk):
    """SYS.CORE1_MODE follows the enables; PDM (last output) and the EQ-worker outputs (from 2) exclude each other."""
    n = profile.num_output_channels
    pdm, eq_out = n - 1, 2
    saved = _enables(dev, n)
    mode = lambda: dev.pget_u8("SYS.CORE1_MODE")  # noqa: E731
    conflict = lambda o: dev.pget_u8("ROUTE.OUTPUT_ENABLE_CONFLICT", o)  # noqa: E731
    try:
        for i in range(2, n):
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 0, i)
        chk.eq(mode(), CORE1_IDLE, "mode IDLE when only 0/1 active")

        dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, eq_out)
        chk.eq(mode(), CORE1_EQ_WORKER, "mode EQ_WORKER after enabling out2")
        chk.eq(conflict(pdm), 1, "OUTPUT_ENABLE_CONFLICT predicts the PDM conflict")
        chk.eq(conflict(3), 0, "no conflict between EQ-worker outputs")
        expect_stall(chk, lambda: dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, pdm), "VC_ERR_STATE",
                     "PDM enable refused by the interlock")
        chk.eq(dev.pget_u8("ROUTE.OUTPUT_ENABLE", pdm), 0, "PDM still disabled")

        for i in range(2, n):
            dev.pset_u8("ROUTE.OUTPUT_ENABLE", 0, i)
        dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, pdm)
        chk.eq(mode(), CORE1_PDM, "mode PDM after enabling PDM")
        chk.eq(conflict(eq_out), 1, "OUTPUT_ENABLE_CONFLICT predicts the EQ conflict while PDM is on")
        expect_stall(chk, lambda: dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, eq_out), "VC_ERR_STATE",
                     "EQ-worker enable refused by the interlock")
        chk.eq(dev.pget_u8("ROUTE.OUTPUT_ENABLE", eq_out), 0, "EQ-worker output still disabled")
        expect_stall(chk, lambda: dev.pget("ROUTE.OUTPUT_ENABLE_CONFLICT", n, 1), "VC_ERR_ADDRESS",
                     "conflict query past the last output")
    finally:
        _set_enables(dev, saved)


# --- Output pins ------------------------------------------------------------

def _pins(dev, profile):
    return [dev.pget_u8("OUTPUT.PIN", i) for i in range(profile.num_pin_outputs)]


def _restore_pins(dev, profile, saved):
    """Put back any output pin a refused-by-spec SET moved anyway, then wait out the restart."""
    for i, (now, want) in enumerate(zip(_pins(dev, profile), saved)):
        if now != want:
            dev.pset_u8("OUTPUT.PIN", want, i)
    settle(dev)


@test("outputs", mutating=True)
def output_pin_status_codes(dev, profile, chk):
    """OUTPUT.PIN refusals carry their codes and no pin moves.
    Bad pin output VC_ERR_ADDRESS, invalid GPIO VC_ERR_PIN_INVALID, GPIO held elsewhere VC_ERR_PIN_IN_USE."""
    saved = _pins(dev, profile)
    npin = profile.num_pin_outputs
    try:
        expect_stall(chk, lambda: dev.pset_u8("OUTPUT.PIN", saved[0], npin), "VC_ERR_ADDRESS",
                     f"SET pin output {npin}")
        for g in invalid_gpios(profile):
            expect_stall(chk, lambda: dev.pset_u8("OUTPUT.PIN", g, 0), "VC_ERR_PIN_INVALID", f"GPIO {g}")
        expect_stall(chk, lambda: dev.pset_u8("OUTPUT.PIN", saved[1], 0), "VC_ERR_PIN_IN_USE",
                     f"GPIO {saved[1]} held by output 1")
        expect_stall(chk, lambda: dev.pget("OUTPUT.PIN", npin, 1), "VC_ERR_ADDRESS", f"GET pin output {npin}")
        chk.eq(_pins(dev, profile), saved, "no output pin moved")
    finally:
        _restore_pins(dev, profile, saved)


@test("outputs", mutating=True)
def output_pin_move_roundtrip(dev, profile, chk):
    """OUTPUT.PIN moves S/PDIF output 0 to a free GPIO and back (each a synchronized restart)."""
    owners = gpio_owners(dev, profile)
    free = free_gpio(dev, profile, prefer=[p for p in (16, 17, 18, 19, 20, 21, 22, 26, 27) if p not in owners])
    if free is None:
        chk.note("no free GPIO available to test pin move; skipping move")
        return
    orig = dev.pget_u8("OUTPUT.PIN", 0)
    pin0 = lambda: dev.pget_u8("OUTPUT.PIN", 0)  # noqa: E731
    try:
        expect_result(chk, lambda: dev.pset_u8("OUTPUT.PIN", free, 0), "VC_OK", f"move out0 -> GPIO {free}")
        chk.eq(wait_for(pin0, free), free, "pin reflects move")
        expect_result(chk, lambda: dev.pset_u8("OUTPUT.PIN", free, 0), "VC_OK", "same-pin SET is a no-op")
    finally:
        dev.pset_u8("OUTPUT.PIN", orig, 0)
        settle(dev)
        chk.eq(wait_for(pin0, orig), orig, "pin restored")


# --- Output type (S/PDIF <-> I2S) -------------------------------------------

@test("outputs", mutating=True)
def output_type_switch_roundtrip(dev, profile, chk):
    """OUTPUT.TYPE: slot 0 S/PDIF -> I2S -> back; the deferred switch applies and the device stays responsive."""
    orig = dev.pget_u8("OUTPUT.TYPE", 0)
    target = TYPE_I2S if orig == TYPE_SPDIF else TYPE_SPDIF
    try:
        expect_result(chk, lambda: dev.pset_u8("OUTPUT.TYPE", target, 0), "VC_OK", f"switch slot0 -> {target}")
        chk.ok(poll_type(dev, 0, target), f"slot0 type became {target} (deferred)")
        chk.ok(dev.is_alive(), "device responsive after type switch")
    finally:
        chk.ok(set_type(dev, 0, orig), "slot0 type restored")


# --- I2S BCK / MCK ----------------------------------------------------------

@test("outputs", mutating=True)
def i2s_bck_pin(dev, profile, chk):
    """CLOCK.I2S_BCK_PIN role 0 refuses invalid pairs and moves to a free pair and back.
    A pair with an invalid GPIO is VC_ERR_PIN_INVALID; a move while a slot is I2S is VC_ERR_STATE."""
    orig = dev.pget_u8("CLOCK.I2S_BCK_PIN", 0)
    bad = invalid_gpios(profile)
    for b in (g - 1 for g in bad if g - 1 not in bad):
        expect_stall(chk, lambda: dev.pset_u8("CLOCK.I2S_BCK_PIN", b, 0), "VC_ERR_PIN_INVALID",
                     f"BCK {b} (LRCLK {b + 1} invalid)")
    chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 0), orig, "BCK unchanged by refused SETs")
    if any(dev.pget_u8("OUTPUT.TYPE", s) == TYPE_I2S for s in range(profile.num_spdif)):
        cand = next(p for p in (16, 18) if p not in (orig, orig + 1))
        expect_stall(chk, lambda: dev.pset_u8("CLOCK.I2S_BCK_PIN", cand, 0), "VC_ERR_STATE",
                     "BCK move refused while an I2S slot is active")
        return
    owners = gpio_owners(dev, profile)
    others = held_by_others(owners, (orig, orig + 1), "i2s_bck0")
    if others:
        chk.note(f"BCK pair {orig}/{orig + 1} is also held by {sorted(others)}; not moving what cannot go back")
        return
    taken = set(owners)
    top = MAX_GPIO[profile.platform_id]
    free = free_gpio(dev, profile, prefer=[p for p in (16, 18, 20, 26)
                                           if p not in taken and p + 1 not in taken and p + 1 <= top])
    if free is None or free + 1 in busy_gpios(dev, profile):
        chk.note("no free BCK pin pair; skipping move")
        return
    try:
        expect_result(chk, lambda: dev.pset_u8("CLOCK.I2S_BCK_PIN", free, 0), "VC_OK", f"BCK move -> {free}")
        chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 0), free, "BCK pin reflects move")
    finally:
        dev.pset_u8("CLOCK.I2S_BCK_PIN", orig, 0)
        settle(dev)
        chk.eq(dev.pget_u8("CLOCK.I2S_BCK_PIN", 0), orig, "BCK pin restored")


@test("outputs", mutating=True)
def mck_enable_and_multiplier(dev, profile, chk):
    """CLOCK.MCK_ENABLE and CLOCK.MCK_MULTIPLIER round-trip; MCK_PIN is refused while MCK runs.
    Multiplier 0 is 128x and 1 is 256x, 2 is VC_ERR_VALUE; a pin SET while enabled is VC_ERR_STATE."""
    u8 = dev.pget_u8
    en0, mult0, pin0 = u8("CLOCK.MCK_ENABLE"), u8("CLOCK.MCK_MULTIPLIER"), u8("CLOCK.MCK_PIN")
    try:
        for m in (0, 1):
            expect_result(chk, lambda: dev.pset_u8("CLOCK.MCK_MULTIPLIER", m), "VC_OK", f"mult {m} accepted")
            chk.eq(u8("CLOCK.MCK_MULTIPLIER"), m, f"mult reads {m}")
        expect_stall(chk, lambda: dev.pset_u8("CLOCK.MCK_MULTIPLIER", 2), "VC_ERR_VALUE", "mult 2")
        chk.eq(u8("CLOCK.MCK_MULTIPLIER"), 1, "mult unchanged after bad set")
        expect_result(chk, lambda: dev.pset_u8("CLOCK.MCK_ENABLE", 1), "VC_OK", "MCK enable accepted")
        chk.eq(u8("CLOCK.MCK_ENABLE"), 1, "MCK enabled")
        expect_stall(chk, lambda: dev.pset_u8("CLOCK.MCK_PIN", 21), "VC_ERR_STATE",
                     "MCK pin move refused while enabled")
        chk.eq(u8("CLOCK.MCK_PIN"), pin0, "MCK pin unchanged")
        expect_result(chk, lambda: dev.pset_u8("CLOCK.MCK_ENABLE", 0), "VC_OK", "MCK disable accepted")
        chk.eq(u8("CLOCK.MCK_ENABLE"), 0, "MCK disabled")
    finally:
        dev.pset_u8("CLOCK.MCK_ENABLE", en0)
        dev.pset_u8("CLOCK.MCK_MULTIPLIER", mult0)
        chk.eq((u8("CLOCK.MCK_ENABLE"), u8("CLOCK.MCK_MULTIPLIER")), (en0, mult0), "MCK restored")


# --- Output latency ---------------------------------------------------------

LAT_LOW, LAT_2MS, LAT_4MS = 1, 3, 5
LAT_STREAM_ONLY, LAT_AVAILABLE = 0x01, 0x80


def _latency(dev):
    req, eff, eff_us, frames, batch, pending, count, _ = struct.unpack(
        "<BBHHHBBH", dev.pget("OUTPUT.LATENCY_STATUS", length=12))
    return dict(requested=req, effective=eff, effective_us=eff_us, target_frames=frames,
                batch_frames=batch, pending=pending, count=count)


def _latency_options(dev, count):
    raw = dev.pget("OUTPUT.LATENCY_OPTIONS", length=8 * count)
    return [dict(zip(("id", "flags", "latency_us", "batch_us", "effective_us"),
                     struct.unpack_from("<BBHHH", raw, 8 * i))) for i in range(count)]


def _set_latency(dev, opt_id, timeout_s=8.0):
    """The main loop applies the SET inside a flash bracket; poll until nothing is pending."""
    dev.pset_u8("OUTPUT.LATENCY", opt_id)
    st = wait_for(lambda: _latency(dev), lambda s: not s["pending"], timeout_s, poll_s=0.1)
    return st if st is not None else _latency(dev)


@test("outputs")
def output_latency_get(dev, profile, chk):
    """OUTPUT.LATENCY_STATUS and LATENCY_OPTIONS agree with each other and with the sample rate."""
    st = _latency(dev)
    rate = sample_rate(dev)
    opts = _latency_options(dev, st["count"])
    ids = [o["id"] for o in opts]
    chk.eq(len(set(ids)), len(ids), "option IDs unique")
    chk.ok(st["requested"] in ids and st["effective"] in ids, "requested and effective are listed")
    chk.ok(0 not in ids, "0 is never an option ID")
    chk.ok(abs(st["target_frames"] - st["effective_us"] * rate / 1e6) <= 1.0,
           f"target {st['target_frames']} frames matches {st['effective_us']} us at {rate} Hz")
    eff = next(o for o in opts if o["id"] == st["effective"])
    chk.eq(eff["effective_us"], st["effective_us"], "table effective_us matches status for the option in force")
    chk.ok(st["batch_frames"] < st["target_frames"], "input batch below the target")
    if dev.pget_u8("INPUT.SOURCE") == 0:   # USB: stream-only options fall back
        for o in opts:
            if o["flags"] & LAT_STREAM_ONLY:
                chk.ok(not o["flags"] & LAT_AVAILABLE, f"option {o['id']} unavailable on USB")


@test("outputs", mutating=True, flash=3)
def output_latency_roundtrip(dev, profile, chk):
    """OUTPUT.LATENCY applies options, falls back on USB, and refuses unknown IDs with VC_ERR_VALUE (flash)."""
    orig = _latency(dev)["requested"]
    try:
        st = _set_latency(dev, LAT_2MS)
        chk.eq((st["requested"], st["effective"]), (LAT_2MS, LAT_2MS), "2 ms applied")
        if dev.pget_u8("INPUT.SOURCE") == 0:
            st = _set_latency(dev, LAT_LOW)
            chk.eq((st["requested"], st["effective"]), (LAT_LOW, LAT_2MS),
                   "low latency on USB stores the request and runs 2 ms")
        before = _latency(dev)
        for bad in (0x00, 0x63):
            expect_stall(chk, lambda: dev.pset_u8("OUTPUT.LATENCY", bad), "VC_ERR_VALUE", f"ID {bad}")
            time.sleep(0.2)
            chk.eq(_latency(dev), before, f"ID {bad} left the latency alone")
    finally:
        _set_latency(dev, orig)
