"""
example_filter.py  --  Example user module for tools/compare_filter.py.

Shows all three supported interfaces.  The active one here is `user_process`,
which mirrors DSPi's firmware SVF inner loop directly (see
firmware/DSPi/dsp_pipeline.c:dsp_compute_coefficients and the SVF branch of
dsp_process_channel_block).  Swap in your own coefficient or inner-loop
code to test changes.

Run:
    python tools/compare_filter.py --user tools/example_filter.py
    python tools/compare_filter.py --user tools/example_filter.py \
        --plot type=peaking,fc=1000,Q=1,gain=6
"""

import numpy as np


# ---------------------------------------------------------------------------
# SVF (Cytomic SvfLinearTrapOptimised2 / Simper 2013) coefficient formulas,
# matching dsp_compute_coefficients() in firmware/DSPi/dsp_pipeline.c.
# ---------------------------------------------------------------------------

def svf_coefficients(filter_type, fc, Q, gain_db, Fs):
    """Return (a1, a2, a3, m0, m1, m2) for the Cytomic linear-trapezoidal SVF."""
    A = 10.0 ** (gain_db / 40.0)
    g = np.tan(np.pi * fc / Fs)

    ft = filter_type.lower()
    if ft == 'lowpass':
        k = 1.0 / Q
        a1 = 1.0 / (1 + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        m0 = 0.0
        m1 = 0.0
        m2 = 1.0
    elif ft == 'highpass':
        k = 1.0 / Q
        a1 = 1.0 / (1 + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        m0 = 1.0
        m1 = -k
        m2 = -1.0
    elif ft == 'bandpass':
        k = 1.0 / Q
        a1 = 1.0 / (1 + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        m0 = 0.0
        m1 = 1.0
        m2 = 0.0
    elif ft == 'notch':
        k = 1.0 / Q
        a1 = 1.0 / (1 + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        m0 = 1.0
        m1 = -k
        m2 = 0.0
    elif ft == 'peaking':
        k = 1.0 / Q
        a1 = 1.0 / (1 + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        m0 = 1.0
        m1 = k * (A * A - 1.0)
        m2 = 0.0
    elif ft == 'lowshelf':
        # Per Cytomic: g_ls = g / sqrt(A); k_ls = 1/(Q * sqrt(A))
        gl = g / np.sqrt(A)
        kl = 1.0 / (Q * np.sqrt(A))
        a1 = 1.0 / (1 + gl * (gl + kl))
        a2 = gl * a1
        a3 = gl * a2
        m0 = 1.0
        m1 = kl * (A - 1.0)
        m2 = A * A - 1.0
    elif ft == 'highshelf':
        gh = g * np.sqrt(A)
        kh = 1.0 / (Q * np.sqrt(A))
        a1 = 1.0 / (1 + gh * (gh + kh))
        a2 = gh * a1
        a3 = gh * a2
        m0 = A * A
        m1 = kh * (1.0 - A) * A
        m2 = 1.0 - A * A
    else:
        raise ValueError(f"Unknown filter type: {filter_type}")

    return a1, a2, a3, m0, m1, m2


# ---------------------------------------------------------------------------
# Interface #1 — `user_process`: full sample-by-sample processing.
# Exercises both coefficient math AND the inner-loop DSP.  This is what the
# firmware does on RP2350 for bands below Fs/7.5.
# ---------------------------------------------------------------------------

def user_process(filter_type, fc, Q, gain_db, Fs, x):
    a1, a2, a3, m0, m1, m2 = svf_coefficients(filter_type, fc, Q, gain_db, Fs)

    y = np.empty_like(x)
    ic1 = 0.0
    ic2 = 0.0
    for n in range(len(x)):
        v0 = x[n]
        v3 = v0 - ic2
        v1_new = a1 * ic1 + a2 * v3
        v2_new = ic2 + a2 * ic1 + a3 * v3
        ic1 = 2.0 * v1_new - ic1
        ic2 = 2.0 * v2_new - ic2
        y[n] = m0 * v0 + m1 * v1_new + m2 * v2_new
    return y


# ---------------------------------------------------------------------------
# Interface #2 — `user_coefficients`: coefficient-only (faster, skips
# the inner loop).  Uncomment to use instead of user_process.
# ---------------------------------------------------------------------------

# def user_coefficients(filter_type, fc, Q, gain_db, Fs):
#     a1, a2, a3, m0, m1, m2 = svf_coefficients(filter_type, fc, Q, gain_db, Fs)
#     return {'kind': 'svf_simper',
#             'a': [a1, a2, a3],
#             'm': [m0, m1, m2]}


# ---------------------------------------------------------------------------
# Interface #3 — `user_response`: return H(e^{jw}) directly.  Uncomment to
# bypass both the coefficient conversion and the inner-loop paths.
# ---------------------------------------------------------------------------

# def user_response(filter_type, fc, Q, gain_db, Fs, w):
#     # e.g. analytic H(z) from some alternative derivation
#     ...
#     return H
