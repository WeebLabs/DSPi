"""
Random valid live-parameter writes, shared by the stress test and the latency soak
(tools/latency_bench/soak.py). Every action is a live, non-flash SET whose
effect a global snapshot restore undoes.
"""

import struct

from .device import PagedStall


def live_param_actions(dev, profile, rng):
    """(name, callable) pairs; each callable performs one random valid SET.
    The rng draw order per action is fixed, so a seed replays the same writes."""
    ni, no, nc, nb = (profile.num_input_channels, profile.num_output_channels,
                      profile.num_channels, profile.band_ceiling)

    def eq_set():
        ch, band = rng.randrange(nc), rng.randrange(nb)
        ftype, freq, q, gain = rng.randrange(8), rng.uniform(20, 18000), rng.uniform(0.2, 10), rng.uniform(-15, 15)
        addr = (ch << 8) | band
        qp = dev.pget("FILTER.BAND", addr, 16)[2:4]   # keep the stored Linkwitz qp
        try:
            dev.pset("FILTER.BAND", struct.pack("<BB", ftype, 0) + qp + struct.pack("<fff", freq, q, gain), addr)
        except PagedStall as s:
            # One pending band slot: a write while the last is unapplied is dropped, as on 1.1.6.
            if s.result_name != "VC_ERR_BUSY":
                raise

    def matrix_set():
        inp, out, en, ph, gain = (rng.randrange(ni), rng.randrange(no), rng.randrange(2), rng.randrange(2),
                                  rng.uniform(-40, 12))
        dev.pset("ROUTE.MATRIX_ROUTE", struct.pack("<BBHf", en, ph, 0, gain), (inp << 8) | out)

    def per_index(cmd, lo, hi, count):
        # The value is drawn before the index, as in the 1.1.6 form of these actions.
        def act():
            v = rng.uniform(lo, hi)
            dev.pset_f32(cmd, v, rng.randrange(count))
        return act

    return [
        ("master_volume", lambda: dev.pset_f32("ROUTE.MASTER_VOLUME", rng.uniform(-60, 0))),
        ("user_volume", lambda: dev.pset_f32("ROUTE.USER_VOLUME", rng.uniform(-60, 0))),
        ("preamp", per_index("ROUTE.PREAMP", -20, 20, ni)),
        ("loudness_ref", lambda: dev.pset_f32("DYN.LOUDNESS_REF", rng.uniform(40, 100))),
        ("loudness_intensity", lambda: dev.pset_f32("DYN.LOUDNESS_INTENSITY", rng.uniform(0, 200))),
        ("crossfeed_freq", lambda: dev.pset_f32("FX.CROSSFEED_FREQ", rng.uniform(500, 2000))),
        ("crossfeed_feed", lambda: dev.pset_f32("FX.CROSSFEED_FEED", rng.uniform(0, 15))),
        ("leveller_amount", lambda: dev.pset_f32("DYN.LEVELLER_AMOUNT", rng.uniform(0, 100))),
        ("leveller_max_gain", lambda: dev.pset_f32("DYN.LEVELLER_MAX_GAIN", rng.uniform(0, 35))),
        ("leveller_gate", lambda: dev.pset_f32("DYN.LEVELLER_GATE", rng.uniform(-96, 0))),
        ("leveller_speed", lambda: dev.pset_u8("DYN.LEVELLER_SPEED", rng.randrange(3))),
        ("output_gain", per_index("ROUTE.OUTPUT_GAIN", -30, 12, no)),
        ("output_delay", per_index("ROUTE.OUTPUT_DELAY", 0, 40, no)),
        ("eq", eq_set),
        ("matrix", matrix_set),
    ]
