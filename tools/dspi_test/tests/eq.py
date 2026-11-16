"""
EQ group: FILTER.BAND type storage and raw parameter storage.
Band layout, addressing, bypass and refusals are covered by paged_filter; preamp and delay by paged_route.
"""

from ..framework import test
from ..helpers import band_addr, read_band, set_retry_busy, wait_band, write_band

# Documented FilterType values for EQ bands (commands.md section 4.7); 32..63 are crossover types.
PEAKING = 1
EQ_TYPES = range(14)
XOVER_TYPE = 32


def _restore_band(dev, ch, band, raw):
    set_retry_busy(dev, "FILTER.BAND", raw, band_addr(ch, band))
    wait_band(dev, ch, band, raw)


@test("eq", mutating=True)
def eq_all_filter_types(dev, profile, chk):
    """Every documented EQ type 0-13 stores and reads back on (ch0, band0); a crossover type in an EQ band is stored verbatim."""
    saved = read_band(dev, 0, 0).pack()
    try:
        for t in EQ_TYPES:
            write_band(dev, 0, 0, t, 500.0, 0.707, 3.0, wait=True)
            chk.eq(read_band(dev, 0, 0).type, t, f"filter type {t}")
        # The device does not check the type against the band kind; only the audio path flattens it.
        write_band(dev, 0, 0, XOVER_TYPE, 500.0, 0.707, 3.0, wait=True)
        chk.eq(read_band(dev, 0, 0).type, XOVER_TYPE, "crossover type 32 in an EQ band stored verbatim")
    finally:
        _restore_band(dev, 0, 0, saved)


@test("eq", mutating=True)
def eq_freq_q_stored_raw_quirk(dev, profile, chk):
    """Quirk: freq and Q clamp only the running copy; the stored band (and GET) keep the RAW values."""
    saved = read_band(dev, 0, 1).pack()
    try:
        # A non-flat band, so the coefficient clamp path actually runs.
        write_band(dev, 0, 1, PEAKING, 1.0, 50.0, 6.0, wait=True)
        b = read_band(dev, 0, 1)
        chk.approx(b.freq, 1.0, 1e-3, "freq stored RAW (not clamped to 10)")
        chk.approx(b.q, 50.0, 1e-3, "Q stored RAW (not clamped to 20)")
        write_band(dev, 0, 1, PEAKING, 1e6, 0.001, 6.0, wait=True)
        b = read_band(dev, 0, 1)
        chk.approx(b.freq, 1e6, 1.0, "freq stored RAW (not clamped to 0.45 Fs)")
        chk.approx(b.q, 0.001, 1e-4, "Q stored RAW (not clamped to 0.1)")
    finally:
        _restore_band(dev, 0, 1, saved)
