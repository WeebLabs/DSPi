"""
Stress / fuzz group.

  * stress_random_valid_sets — a storm of random *valid* parameter writes with
    liveness checks; the device must stay responsive and coherent.
  * stress_wvalue_fuzz       — pure-read GETs hammered with garbage wValue/length;
    must respond or STALL gracefully, never crash.
  * stress_payload_fuzz      — safe live-param SETs with random-length payloads;
    must no-op/clamp/accept, never crash.

Scale with --stress N (default light pass).  All opcodes here are live, non-flash
params; flash writers (incl. 0x52 REQ_SAVE_OUTPUT_CONFIG), pin/type/input
switches, and 0xF0 are excluded.  Global snapshot restore returns all mutated
params afterwards.
"""

import random
import struct

from ..device import OP, OPCODE_NAME, Stall, Disconnected
from ..framework import test

# Opcodes that must never be fuzzed: actions / SET-via-wValue / flash / bootloader.
_FUZZ_GET_DENY = {
    0xF0,  # ENTER_BOOTLOADER
    0x52, 0x51, 0x53,             # SAVE_OUTPUT_CONFIG / SAVE_PARAMS / FACTORY_RESET (flash/action)
    0x90, 0x91, 0x92,             # preset save/load/delete
    0xD6,                         # save master volume
    0x83, 0xB1, 0xB3,             # clear clips / reset stats (mutate)
    0x7C, 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xE4, 0xEC,  # SET-via-wValue / test pulse
}


def _iters(profile, default):
    n = getattr(profile, "stress_iters", 0) or 0
    return max(n, default)


@test("stress", mutating=True)
def stress_random_valid_sets(dev, profile, chk):
    """A storm of random valid SETs across the live parameter surface; device stays coherent."""
    rng = random.Random(0xD597)
    ni, no, nc, nb = (profile.num_input_channels, profile.num_output_channels,
                      profile.num_channels, profile.band_ceiling)
    lg = dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE)
    iters = _iters(profile, 60)

    def eq_set():
        ch, band = rng.randrange(nc), rng.randrange(nb)
        dev.set(OP.SET_EQ_PARAM, struct.pack("<BBBBfff", ch, band, rng.randrange(8), 0,
                                             rng.uniform(20, 18000), rng.uniform(0.2, 10), rng.uniform(-15, 15)))

    def matrix_set():
        dev.set(OP.SET_MATRIX_ROUTE, struct.pack("<BBBBf", rng.randrange(ni), rng.randrange(no),
                                                 rng.randrange(2), rng.randrange(2), rng.uniform(-40, 12)))

    actions = [
        lambda: dev.set_f32(OP.SET_MASTER_VOLUME, rng.uniform(-60, 0)),
        lambda: dev.set_f32(OP.SET_USER_VOLUME, rng.uniform(-60, 0)),
        lambda: dev.set_f32(OP.SET_PREAMP_CH, rng.uniform(-20, 20), wvalue=rng.randrange(ni)),
        lambda: dev.set_f32(OP.SET_LOUDNESS_REF, rng.uniform(40, 100)),
        lambda: dev.set_f32(OP.SET_LOUDNESS_INTENSITY, rng.uniform(0, 200)),
        lambda: dev.set_f32(OP.SET_CROSSFEED_FREQ, rng.uniform(500, 2000)),
        lambda: dev.set_f32(OP.SET_CROSSFEED_FEED, rng.uniform(0, 15)),
        lambda: dev.set_f32(OP.SET_LEVELLER_AMOUNT, rng.uniform(0, 100)),
        lambda: dev.set_f32(OP.SET_LEVELLER_MAX_GAIN, rng.uniform(0, 35)),
        lambda: dev.set_f32(OP.SET_LEVELLER_GATE, rng.uniform(-96, 0)),
        lambda: dev.set_u8(OP.SET_LEVELLER_SPEED, rng.randrange(3)),
        lambda: dev.set_f32(OP.SET_OUTPUT_GAIN, rng.uniform(-30, 12), wvalue=rng.randrange(no)),
        lambda: dev.set_f32(OP.SET_OUTPUT_DELAY, rng.uniform(0, 40), wvalue=rng.randrange(no)),
        eq_set, matrix_set,
    ]
    try:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, 0)
        for i in range(iters):
            rng.choice(actions)()
            if i % 12 == 0 and not dev.is_alive():
                chk.ok(False, f"device unresponsive mid-storm at iter {i}")
                return
        chk.ok(dev.is_alive(), "device responsive after random-set storm")
        # Coherent read after the storm.
        chk.in_range(dev.get_f32(OP.GET_MASTER_VOLUME), -128.0, 0.0, "master vol sane after storm")
        chk.note(f"{iters} random valid SETs survived")
    finally:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, lg)


@test("stress", mutating=True)
def stress_wvalue_fuzz(dev, profile, chk):
    """Pure-read GET opcodes hammered with random wValue/length must STALL or return, never crash."""
    rng = random.Random(0x6A11)
    read_ops = [op for op, name in OPCODE_NAME.items()
                if name.startswith("REQ_GET_") or name.startswith("REQ_PRESET_GET_")]
    iters = _iters(profile, 200)
    for i in range(iters):
        op = rng.choice(read_ops)
        wv = rng.randrange(0x10000)
        ln = rng.choice([1, 2, 4, 8, 16, 24, 32, 64])
        try:
            dev.get(op, ln, wvalue=wv)
        except Stall:
            pass  # graceful refusal is fine
        except Disconnected:
            raise  # real crash -> ERROR/abort
        if i % 40 == 0 and not dev.is_alive():
            chk.ok(False, f"device unresponsive during GET fuzz at iter {i}")
            return
    chk.ok(dev.is_alive(), "device responsive after wValue fuzz")
    chk.note(f"{iters} fuzzed GETs survived")


@test("stress", mutating=True)
def stress_payload_fuzz(dev, profile, chk):
    """Safe live-param SETs with random-length payloads must no-op/clamp/accept, never crash."""
    rng = random.Random(0x9E37)
    # (opcode, max valid payload) for genuinely safe live params only.
    safe_sets = [OP.SET_MASTER_VOLUME, OP.SET_USER_VOLUME, OP.SET_LOUDNESS_REF,
                 OP.SET_LOUDNESS_INTENSITY, OP.SET_CROSSFEED_FREQ, OP.SET_LEVELLER_AMOUNT,
                 OP.SET_LEVELLER_GATE, OP.SET_EQ_PARAM, OP.SET_MATRIX_ROUTE, OP.SET_DELAY,
                 OP.SET_BYPASS, OP.SET_LOUDNESS, OP.SET_USER_MUTE]
    lg = dev.get_u8(OP.GET_LG_SOUND_SYNC_ENABLE)
    iters = _iters(profile, 150)
    try:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, 0)
        for i in range(iters):
            op = rng.choice(safe_sets)
            ln = rng.randrange(0, 65)  # 0..64 valid lengths (65+ would STALL, tested elsewhere)
            payload = bytes(rng.randrange(256) for _ in range(ln))
            wv = rng.randrange(0, 12)
            try:
                dev.set(op, payload, wvalue=wv)
            except Stall:
                pass
            except Disconnected:
                raise
            if i % 30 == 0 and not dev.is_alive():
                chk.ok(False, f"device unresponsive during payload fuzz at iter {i}")
                return
        chk.ok(dev.is_alive(), "device responsive after payload fuzz")
        chk.note(f"{iters} fuzzed SETs survived")
    finally:
        dev.set_u8(OP.SET_LG_SOUND_SYNC_ENABLE, lg)
