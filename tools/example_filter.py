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
    """Return (a1, a2, a3, m0, m1, m2) for the linear-trapezoidal SVF.

    Mirrors the firmware exactly (firmware/DSPi/dsp_pipeline.c:94-129,
    plus PR #39 for notch).  Start with g = tan(π·fc/Fs) and k = 1/Q,
    then apply per-type modifications to g and/or k, derive the integrator
    coefficients, and pick the m coefficients for the desired output shape.
    All configurations match the RBJ Audio-EQ-Cookbook response to machine
    precision — verified by the compare_filter.py sweep.
    """
    A = 10.0 ** (gain_db / 40.0)
    ft = filter_type.lower()

    # Defaults (firmware lines 97-98)
    g = np.tan(np.pi * fc / Fs)
    k = 1.0 / Q

    # Per-type g/k modifications (firmware lines 100-115)
    if ft == 'peaking':
        k = 1.0 / (Q * A)
    elif ft == 'lowshelf':
        g = g / np.sqrt(A)
    elif ft == 'highshelf':
        g = g * np.sqrt(A)
    elif ft == 'bandpass':
        # Not in firmware — tool-only.  Scale m1 below to normalise the
        # SVF BP peak gain (= Q) to 1, matching RBJ's constant-0-dB-peak
        # bandpass.
        pass
    # lowpass, highpass, notch: no modification

    # Integrator coefficients (firmware lines 117-119)
    a1 = 1.0 / (1.0 + g * (g + k))
    a2 = g * a1
    a3 = g * a2

    # Output mix (firmware lines 122-129, plus PR #39 for notch)
    if ft == 'lowpass':
        m0, m1, m2 = 0.0, 0.0, 1.0
    elif ft == 'highpass':
        m0, m1, m2 = 1.0, -k, -1.0
    elif ft == 'peaking':
        m0, m1, m2 = 1.0, k * (A * A - 1.0), 0.0
    elif ft == 'lowshelf':
        m0, m1, m2 = 1.0, k * (A - 1.0), A * A - 1.0
    elif ft == 'highshelf':
        m0, m1, m2 = A * A, k * (1.0 - A) * A, 1.0 - A * A
    elif ft == 'notch':
        m0, m1, m2 = 1.0, -k, 0.0
    elif ft == 'bandpass':
        # Tool-only: normalise peak to unity (RBJ constant-skirt-gain BP).
        m0, m1, m2 = 0.0, k, 0.0
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
