"""
Random valid live-parameter writes, shared by the stress test and the latency
soak (tools/latency_bench/soak.py). Every action is a live, non-flash SET whose
effect a global snapshot restore undoes.
"""

import struct

from .device import OP


def live_param_actions(dev, profile, rng):
    """(name, callable) pairs; each callable performs one random valid SET."""
    ni, no, nc, nb = (profile.num_input_channels, profile.num_output_channels,
                      profile.num_channels, profile.band_ceiling)

    def eq_set():
        ch, band = rng.randrange(nc), rng.randrange(nb)
        dev.set(OP.SET_EQ_PARAM, struct.pack("<BBBBfff", ch, band, rng.randrange(8), 0,
                                             rng.uniform(20, 18000), rng.uniform(0.2, 10), rng.uniform(-15, 15)))

    def matrix_set():
        dev.set(OP.SET_MATRIX_ROUTE, struct.pack("<BBBBf", rng.randrange(ni), rng.randrange(no),
                                                 rng.randrange(2), rng.randrange(2), rng.uniform(-40, 12)))

    return [
        ("master_volume", lambda: dev.set_f32(OP.SET_MASTER_VOLUME, rng.uniform(-60, 0))),
        ("user_volume", lambda: dev.set_f32(OP.SET_USER_VOLUME, rng.uniform(-60, 0))),
        ("preamp", lambda: dev.set_f32(OP.SET_PREAMP_CH, rng.uniform(-20, 20), wvalue=rng.randrange(ni))),
        ("loudness_ref", lambda: dev.set_f32(OP.SET_LOUDNESS_REF, rng.uniform(40, 100))),
        ("loudness_intensity", lambda: dev.set_f32(OP.SET_LOUDNESS_INTENSITY, rng.uniform(0, 200))),
        ("crossfeed_freq", lambda: dev.set_f32(OP.SET_CROSSFEED_FREQ, rng.uniform(500, 2000))),
        ("crossfeed_feed", lambda: dev.set_f32(OP.SET_CROSSFEED_FEED, rng.uniform(0, 15))),
        ("leveller_amount", lambda: dev.set_f32(OP.SET_LEVELLER_AMOUNT, rng.uniform(0, 100))),
        ("leveller_max_gain", lambda: dev.set_f32(OP.SET_LEVELLER_MAX_GAIN, rng.uniform(0, 35))),
        ("leveller_gate", lambda: dev.set_f32(OP.SET_LEVELLER_GATE, rng.uniform(-96, 0))),
        ("leveller_speed", lambda: dev.set_u8(OP.SET_LEVELLER_SPEED, rng.randrange(3))),
        ("output_gain", lambda: dev.set_f32(OP.SET_OUTPUT_GAIN, rng.uniform(-30, 12), wvalue=rng.randrange(no))),
        ("output_delay", lambda: dev.set_f32(OP.SET_OUTPUT_DELAY, rng.uniform(0, 40), wvalue=rng.randrange(no))),
        ("eq", eq_set),
        ("matrix", matrix_set),
    ]
