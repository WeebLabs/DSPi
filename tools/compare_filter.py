#!/usr/bin/env python3
"""
compare_filter.py  --  Compare a user-supplied biquad/SVF filter design
against the RBJ cookbook reference across a sweep of types, frequencies,
Q values, and gains.

USAGE
-----
  python tools/compare_filter.py --user path/to/my_filter.py [options]

The user module must define ONE of:

  1) def user_process(filter_type, fc, Q, gain_db, Fs, x):
         # Full processing: applies the filter to input samples `x` and
         # returns output samples of the same length.  Exercises BOTH
         # coefficient computation AND the inner-loop DSP code.  The tool
         # feeds a unit impulse and computes H(f) from the impulse response.
         ...

  2) def user_coefficients(filter_type, fc, Q, gain_db, Fs):
         # Return a dict describing topology and coefficients only.  The
         # tool evaluates H(z) analytically from the coefficients.
         return {'kind': 'biquad',
                 'b': [b0, b1, b2], 'a': [a0, a1, a2]}
         # or
         return {'kind': 'svf_simper',     # Cytomic SvfLinearTrapOptimised2
                 'a': [a1, a2, a3],        # integrator coefs
                 'm': [m0, m1, m2]}        # mixing coefs

  3) def user_response(filter_type, fc, Q, gain_db, Fs, w):
         # w: array of normalised frequencies in rad/sample.
         # Return complex H(e^{jw}).
         ...

If you want to validate both your coefficient math AND your SVF/biquad
inner-loop DSP code end-to-end, use mode (1).  If you only want to check
the coefficient formulas, mode (2) is faster.

filter_type is one of: 'lowpass', 'highpass', 'bandpass', 'notch',
                       'peaking', 'lowshelf', 'highshelf'.

EXAMPLES
--------
  # Full sweep, report any config whose magnitude error exceeds 0.1 dB:
  python tools/compare_filter.py --user tools/example_filter.py

  # Plot a single configuration:
  python tools/compare_filter.py --user tools/example_filter.py \\
      --plot type=peaking,fc=1000,Q=1,gain=6

  # Constrained sweep:
  python tools/compare_filter.py --user tools/example_filter.py \\
      --types peaking,lowshelf --fcs 100,1000,5000 --gains -6,0,6
"""

import argparse
import importlib.util
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except ImportError:
    HAS_PLT = False


# ---------------------------------------------------------------------------
# RBJ reference (Robert Bristow-Johnson, Audio EQ Cookbook)
# ---------------------------------------------------------------------------

def rbj_coefficients(filter_type, fc, Q, gain_db, Fs):
    """Return un-normalised biquad coefficients (b0,b1,b2,a0,a1,a2)."""
    A = 10.0 ** (gain_db / 40.0)
    w0 = 2 * np.pi * fc / Fs
    cs = np.cos(w0)
    sn = np.sin(w0)
    alpha = sn / (2 * Q)
    sqA2 = 2 * np.sqrt(A) * alpha

    ft = filter_type.lower()
    if ft == 'lowpass':
        b0 = (1 - cs) * 0.5
        b1 = 1 - cs
        b2 = (1 - cs) * 0.5
        a0 = 1 + alpha
        a1 = -2 * cs
        a2 = 1 - alpha
    elif ft == 'highpass':
        b0 = (1 + cs) * 0.5
        b1 = -(1 + cs)
        b2 = (1 + cs) * 0.5
        a0 = 1 + alpha
        a1 = -2 * cs
        a2 = 1 - alpha
    elif ft == 'bandpass':
        # Constant 0-dB peak (RBJ "constant skirt gain" variant)
        b0 = alpha
        b1 = 0
        b2 = -alpha
        a0 = 1 + alpha
        a1 = -2 * cs
        a2 = 1 - alpha
    elif ft == 'notch':
        b0 = 1
        b1 = -2 * cs
        b2 = 1
        a0 = 1 + alpha
        a1 = -2 * cs
        a2 = 1 - alpha
    elif ft == 'peaking':
        b0 = 1 + alpha * A
        b1 = -2 * cs
        b2 = 1 - alpha * A
        a0 = 1 + alpha / A
        a1 = -2 * cs
        a2 = 1 - alpha / A
    elif ft == 'lowshelf':
        b0 = A * ((A + 1) - (A - 1) * cs + sqA2)
        b1 = 2 * A * ((A - 1) - (A + 1) * cs)
        b2 = A * ((A + 1) - (A - 1) * cs - sqA2)
        a0 = (A + 1) + (A - 1) * cs + sqA2
        a1 = -2 * ((A - 1) + (A + 1) * cs)
        a2 = (A + 1) + (A - 1) * cs - sqA2
    elif ft == 'highshelf':
        b0 = A * ((A + 1) + (A - 1) * cs + sqA2)
        b1 = -2 * A * ((A - 1) + (A + 1) * cs)
        b2 = A * ((A + 1) + (A - 1) * cs - sqA2)
        a0 = (A + 1) - (A - 1) * cs + sqA2
        a1 = 2 * ((A - 1) - (A + 1) * cs)
        a2 = (A + 1) - (A - 1) * cs - sqA2
    else:
        raise ValueError(f"Unknown filter type: {filter_type}")
    return (b0, b1, b2, a0, a1, a2)


# ---------------------------------------------------------------------------
# Transfer-function evaluators
# ---------------------------------------------------------------------------

def eval_biquad(coefs, w):
    """Evaluate a biquad's frequency response at normalised freqs w (rad/sample).

    coefs = (b0, b1, b2, a0, a1, a2) — may be unnormalised (a0 != 1).
    """
    b0, b1, b2, a0, a1, a2 = coefs
    z1 = np.exp(-1j * w)
    z2 = z1 * z1
    num = b0 + b1 * z1 + b2 * z2
    den = a0 + a1 * z1 + a2 * z2
    return num / den


def svf_simper_to_biquad(a_coefs, m_coefs):
    """Convert Simper-SVF coefs to an equivalent (b, a) biquad, via the
    closed-form state-space → transfer function derivation below.

    Topology (SvfLinearTrapOptimised2, Andrew Simper 2013):

        v3     = v0 - ic2
        v1_new = a1*ic1 + a2*v3
        v2_new = ic2 + a2*ic1 + a3*v3
        ic1    = 2*v1_new - ic1
        ic2    = 2*v2_new - ic2
        output = m0*v0 + m1*v1_new + m2*v2_new

    Deriving the z-domain state-space model (state x = [ic1, ic2]):

        A = [[2*a1 - 1,  -2*a2     ],
             [ 2*a2,      1 - 2*a3 ]]
        B = [[ 2*a2 ],
             [ 2*a3 ]]
        C = [ m1*a1 + m2*a2,  m2*(1 - a3) - m1*a2 ]
        D = m0 + m1*a2 + m2*a3

    Then H(z) = C*(zI - A)^{-1}*B + D.  For a 2×2 A, this is:

        det(zI - A) = z^2 - tr(A)*z + det(A)
                    = z^2 - 2*(a1 - a3)*z + [(2*a1-1)(1-2*a3) + 4*a2^2]

        adj(zI - A)*B  →  numerator polynomial in z, degree <= 1

        H(z) = [D*z^2 + (b1*c1 + b2*c2 - D*tr)*z + (const + D*det)] / [z^2 - tr*z + det]

    where const = 2*a2*(b2*c1 - b1*c2) - b1*c1*(1-2*a3) - b2*c2*(2*a1-1).

    Returned as biquad coefficients referenced to z^{-2} (so the polynomials
    read as b0 + b1*z^{-1} + b2*z^{-2}).
    """
    a1, a2, a3 = a_coefs
    m0, m1, m2 = m_coefs

    tr = 2 * (a1 - a3)
    detA = (2 * a1 - 1) * (1 - 2 * a3) + 4 * a2 * a2

    # State-space: B = [2*a2, 2*a3], C = [c1, c2], D
    b1_B = 2 * a2
    b2_B = 2 * a3
    c1 = m1 * a1 + m2 * a2
    c2 = m2 * (1 - a3) - m1 * a2
    D = m0 + m1 * a2 + m2 * a3

    zcoef = b1_B * c1 + b2_B * c2
    # (C · adj(zI−A) · B) constant term:
    #   −b1·c1·(1−2·a3) − b2·c2·(2·a1−1) + 2·a2·(b1·c2 − b2·c1)
    const = (2 * a2 * (b1_B * c2 - b2_B * c1)
             - b1_B * c1 * (1 - 2 * a3)
             - b2_B * c2 * (2 * a1 - 1))

    # Numerator (descending powers of z): [D, zcoef - D*tr, const + D*detA]
    # Denominator:                       [1, -tr, detA]
    # Convert to z^{-1} form (same coefficients, different interpretation):
    # (b0 + b1 z^{-1} + b2 z^{-2}) / (a0 + a1 z^{-1} + a2 z^{-2})
    b0 = D
    b1 = zcoef - D * tr
    b2 = const + D * detA
    a0 = 1.0
    a1c = -tr
    a2c = detA
    return (b0, b1, b2, a0, a1c, a2c)


def response_from_coefs(coefs_dict, w):
    """Compute H(e^{jw}) from a coefficients dict as produced by the user."""
    kind = coefs_dict['kind']
    if kind == 'biquad':
        b = coefs_dict['b']
        a = coefs_dict['a']
        if len(b) != 3 or len(a) != 3:
            raise ValueError("biquad coefs must have 3 b and 3 a values")
        return eval_biquad(tuple(b) + tuple(a), w)
    if kind == 'svf_simper':
        coefs = svf_simper_to_biquad(coefs_dict['a'], coefs_dict['m'])
        return eval_biquad(coefs, w)
    raise ValueError(f"Unknown topology kind: {kind}")


# ---------------------------------------------------------------------------
# Sanity check: compare analytic SVF response against a time-domain impulse
# response simulation.  Run via --verify-svf to rule out any derivation bugs.
# ---------------------------------------------------------------------------

def svf_impulse_response(a_coefs, m_coefs, N=8192):
    a1, a2, a3 = a_coefs
    m0, m1, m2 = m_coefs
    h = np.zeros(N)
    ic1 = 0.0
    ic2 = 0.0
    for n in range(N):
        v0 = 1.0 if n == 0 else 0.0
        v3 = v0 - ic2
        v1_new = a1 * ic1 + a2 * v3
        v2_new = ic2 + a2 * ic1 + a3 * v3
        ic1 = 2 * v1_new - ic1
        ic2 = 2 * v2_new - ic2
        h[n] = m0 * v0 + m1 * v1_new + m2 * v2_new
    return h


def svf_response_by_impulse(a_coefs, m_coefs, w, N=8192):
    h = svf_impulse_response(a_coefs, m_coefs, N)
    n = np.arange(N)
    # DTFT at w: H(e^jw) = sum h[n] e^{-jwn}
    # Vectorised via matrix-exp (small for reasonable n_freqs).
    return h @ np.exp(-1j * np.outer(n, w))


# ---------------------------------------------------------------------------
# User module loader
# ---------------------------------------------------------------------------

def load_user_module(path):
    spec = importlib.util.spec_from_file_location("user_filter_mod", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load user module from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def response_from_process(user_process, filter_type, fc, Q, gain_db, Fs, w,
                           N=8192):
    """Feed a unit impulse through user_process(), compute H(e^{jw}) via DTFT.

    N is the impulse-response length.  8192 samples = ~170 ms at 48 kHz,
    enough for the slowest practical biquad/SVF pole to decay below noise.
    """
    x = np.zeros(N, dtype=np.float64)
    x[0] = 1.0
    y = np.asarray(user_process(filter_type, fc, Q, gain_db, Fs, x),
                   dtype=np.float64)
    if y.shape[0] != N:
        raise ValueError(
            f"user_process returned {y.shape[0]} samples, expected {N}")
    n = np.arange(N)
    # DTFT of h at each w: H(e^jw) = sum_n h[n] e^{-jwn}
    # Vectorised as y · exp(-j * outer(n, w))
    return y @ np.exp(-1j * np.outer(n, w))


def get_user_response(user_mod, filter_type, fc, Q, gain_db, Fs, w,
                      impulse_length=8192):
    # Prefer the richest interface available.
    if hasattr(user_mod, 'user_process'):
        return response_from_process(user_mod.user_process,
                                     filter_type, fc, Q, gain_db, Fs, w,
                                     N=impulse_length)
    if hasattr(user_mod, 'user_response'):
        return user_mod.user_response(filter_type, fc, Q, gain_db, Fs, w)
    if hasattr(user_mod, 'user_coefficients'):
        coefs = user_mod.user_coefficients(filter_type, fc, Q, gain_db, Fs)
        return response_from_coefs(coefs, w)
    raise AttributeError(
        "user module must define `user_process`, `user_response`, or "
        "`user_coefficients`")


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def compare_at_config(user_mod, filter_type, fc, Q, gain_db, Fs, n_freqs=512,
                      impulse_length=8192):
    f = np.logspace(np.log10(10.0), np.log10(Fs * 0.475), n_freqs)
    w = 2 * np.pi * f / Fs

    ref = eval_biquad(rbj_coefficients(filter_type, fc, Q, gain_db, Fs), w)
    user = get_user_response(user_mod, filter_type, fc, Q, gain_db, Fs, w,
                             impulse_length=impulse_length)

    mag_ref_db = 20 * np.log10(np.abs(ref) + 1e-30)
    mag_user_db = 20 * np.log10(np.abs(user) + 1e-30)
    phase_ref = np.unwrap(np.angle(ref))
    phase_user = np.unwrap(np.angle(user))

    phase_err_deg = np.degrees(np.abs(phase_user - phase_ref))
    # Fold to [0, 180] — direction doesn't matter for "are these the same?"
    phase_err_deg = np.minimum(phase_err_deg, 360 - phase_err_deg)

    return {
        'max_mag_err_db': float(np.max(np.abs(mag_user_db - mag_ref_db))),
        'max_phase_err_deg': float(np.max(phase_err_deg)),
        'f_hz': f,
        'mag_ref_db': mag_ref_db,
        'mag_user_db': mag_user_db,
        'phase_ref_deg': np.degrees(phase_ref),
        'phase_user_deg': np.degrees(phase_user),
    }


# Five disparate (fc_Hz, Q, gain_dB) points per filter type — span low/mid/high
# fc, low/mid/high Q, and (for gain-shaped types) low/mid/high gain.  Used by
# the --quick mode so one command tests every type at representative configs
# without running a full cross-product.
QUICK_CONFIGS = [
    # fc (Hz),  Q,      gain (dB)
    (100.0,     0.707,  -12.0),
    (500.0,     1.0,    -3.0),
    (1500.0,    2.0,     1.0),
    (5000.0,    0.5,     6.0),
    (12000.0,   5.0,    12.0),
]


def run_quick_sweep(user_mod, types, Fs, threshold_db,
                    n_freqs=512, impulse_length=8192):
    header = (f"{'type':<10} {'fc Hz':>10} {'Q':>6} {'gain dB':>8} "
              f"{'mag err':>9} {'phase err':>10}  status")
    print(header)
    print('-' * len(header))

    failures = []
    total = 0
    for ft in types:
        uses_gain = ft in ('peaking', 'lowshelf', 'highshelf')
        for fc, Q, gain in QUICK_CONFIGS:
            g = gain if uses_gain else 0.0
            total += 1
            r = compare_at_config(user_mod, ft, fc, Q, g, Fs, n_freqs,
                                  impulse_length)
            ok = r['max_mag_err_db'] <= threshold_db
            status = 'OK  ' if ok else 'FAIL'
            print(f"{ft:<10} {fc:>10.1f} {Q:>6.2f} {g:>8.1f} "
                  f"{r['max_mag_err_db']:>7.3f}dB "
                  f"{r['max_phase_err_deg']:>8.2f}°   {status}")
            if not ok:
                failures.append({'type': ft, 'fc': fc, 'Q': Q, 'gain_db': g,
                                 'result': r})
    print()
    print(f"{total - len(failures)}/{total} configs within {threshold_db} dB.")
    return failures


def run_sweep(user_mod, types, fcs, Qs, gains_db, Fs, threshold_db,
              n_freqs=512, impulse_length=8192):
    header = (f"{'type':<10} {'fc Hz':>10} {'Q':>6} {'gain dB':>8} "
              f"{'mag err':>9} {'phase err':>10}  status")
    print(header)
    print('-' * len(header))

    failures = []
    total = 0
    for ft in types:
        uses_gain = ft in ('peaking', 'lowshelf', 'highshelf')
        test_gains = gains_db if uses_gain else [0.0]
        for fc in fcs:
            for Q in Qs:
                for g in test_gains:
                    total += 1
                    r = compare_at_config(user_mod, ft, fc, Q, g, Fs,
                                          n_freqs, impulse_length)
                    ok = r['max_mag_err_db'] <= threshold_db
                    status = 'OK  ' if ok else 'FAIL'
                    print(f"{ft:<10} {fc:>10.1f} {Q:>6.2f} {g:>8.1f} "
                          f"{r['max_mag_err_db']:>7.3f}dB "
                          f"{r['max_phase_err_deg']:>8.2f}°   {status}")
                    if not ok:
                        failures.append({
                            'type': ft, 'fc': fc, 'Q': Q, 'gain_db': g,
                            'result': r,
                        })
    print()
    print(f"{total - len(failures)}/{total} configs within {threshold_db} dB.")
    return failures


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def parse_plot_spec(spec):
    kw = {}
    for part in spec.split(','):
        if '=' not in part:
            continue
        k, v = part.split('=', 1)
        k = k.strip()
        v = v.strip()
        if k in ('fc', 'Q', 'gain', 'fs'):
            kw[k] = float(v)
        else:
            kw[k] = v
    return kw


def plot_comparison(user_mod, filter_type, fc, Q, gain_db, Fs, title=None):
    if not HAS_PLT:
        print("matplotlib is not available; cannot plot.", file=sys.stderr)
        sys.exit(2)
    r = compare_at_config(user_mod, filter_type, fc, Q, gain_db, Fs,
                          n_freqs=1024)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    ax1.semilogx(r['f_hz'], r['mag_ref_db'], label='RBJ ref', linewidth=1.8)
    ax1.semilogx(r['f_hz'], r['mag_user_db'], label='user',
                 linewidth=1.2, linestyle='--')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.legend(loc='best')

    err_db = r['mag_user_db'] - r['mag_ref_db']
    ax2.semilogx(r['f_hz'], err_db, color='tab:red', linewidth=1.1)
    ax2.axhline(0, color='0.5', linewidth=0.5)
    ax2.set_ylabel('Mag error (dB)')
    ax2.grid(True, which='both', alpha=0.3)

    ax3.semilogx(r['f_hz'], r['phase_ref_deg'], label='RBJ ref', linewidth=1.8)
    ax3.semilogx(r['f_hz'], r['phase_user_deg'], label='user',
                 linewidth=1.2, linestyle='--')
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_ylabel('Phase (deg)')
    ax3.grid(True, which='both', alpha=0.3)
    ax3.legend(loc='best')

    if title is None:
        title = (f"{filter_type}  fc={fc:g} Hz  Q={Q:g}  gain={gain_db:g} dB"
                 f"  Fs={Fs:g}  (max err {r['max_mag_err_db']:.3f} dB)")
    fig.suptitle(title)
    fig.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_list(s, cast=float):
    return [cast(x.strip()) for x in s.split(',') if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n', 1)[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See the module docstring at the top of this file for the "
               "user-module interface.")
    ap.add_argument('--user', required=True,
                    help='Path to user filter module (.py)')
    ap.add_argument('--fs', type=float, default=48000.0,
                    help='Sample rate (default 48000)')
    ap.add_argument('--types',
                    default='lowpass,highpass,peaking,lowshelf,highshelf,notch,bandpass',
                    help='Comma-separated filter types')
    ap.add_argument('--fcs', default=None,
                    help='Comma-separated center frequencies '
                         '(default: log-spaced 20..18000, 7 points)')
    ap.add_argument('--qs', default='0.5,0.707,1,2,5',
                    help='Comma-separated Q values')
    ap.add_argument('--gains', default='-12,-6,-1,1,6,12',
                    help='Comma-separated gains in dB '
                         '(used for peaking/shelving only)')
    ap.add_argument('--threshold', type=float, default=0.1,
                    help='Maximum acceptable magnitude error in dB')
    ap.add_argument('--n-freqs', type=int, default=512,
                    help='Number of frequency points per config')
    ap.add_argument('--impulse-length', type=int, default=8192,
                    help='Impulse-response length for `user_process` mode '
                         '(default 8192 samples)')
    ap.add_argument('--plot', default=None,
                    help="Plot a single config, e.g. "
                         "'type=peaking,fc=1000,Q=1,gain=6'")
    ap.add_argument('--plot-failures', action='store_true',
                    help='Also plot each config that exceeds the threshold')
    ap.add_argument('--quick', action='store_true',
                    help='Run a curated 5-config spread per filter type '
                         '(one sweep point per type at low/mid/high '
                         'fc × Q × gain) instead of a full cross-product sweep')
    ap.add_argument('--verify-svf', action='store_true',
                    help='Run an extra sanity check comparing the analytic '
                         'SVF → biquad conversion against a time-domain '
                         'impulse simulation')

    args = ap.parse_args(argv)
    user_mod = load_user_module(args.user)

    if args.plot:
        kw = parse_plot_spec(args.plot)
        ft = kw.get('type', 'peaking')
        fs = kw.get('fs', args.fs)
        plot_comparison(user_mod, ft, kw['fc'], kw['Q'],
                        kw.get('gain', 0.0), fs)
        return 0

    if args.verify_svf:
        print("Analytic SVF vs time-domain impulse simulation")
        print("-" * 50)
        # Try a representative SVF config via user module
        try:
            coefs = user_mod.user_coefficients('peaking', 1000.0, 1.0,
                                               6.0, args.fs)
        except AttributeError:
            print("user module does not expose user_coefficients — "
                  "cannot verify SVF path")
            return 0
        if coefs.get('kind') != 'svf_simper':
            print("user topology is not svf_simper; skipping")
            return 0
        f = np.logspace(np.log10(10), np.log10(args.fs * 0.45), 128)
        w = 2 * np.pi * f / args.fs
        H_analytic = response_from_coefs(coefs, w)
        H_impulse = svf_response_by_impulse(coefs['a'], coefs['m'], w)
        diff_db = np.max(np.abs(
            20 * np.log10(np.abs(H_analytic) + 1e-30)
            - 20 * np.log10(np.abs(H_impulse) + 1e-30)))
        print(f"max |analytic − impulse| magnitude error: {diff_db:.5f} dB")
        print()

    types = [t.strip() for t in args.types.split(',') if t.strip()]

    if args.quick:
        print(f"Fs={args.fs:g} Hz  threshold={args.threshold} dB  "
              f"n_freqs/config={args.n_freqs}  (quick mode: "
              f"{len(QUICK_CONFIGS)} configs × {len(types)} types)")
        print()
        failures = run_quick_sweep(user_mod, types, args.fs, args.threshold,
                                   args.n_freqs, args.impulse_length)
    else:
        if args.fcs:
            fcs = parse_list(args.fcs, float)
        else:
            fcs = np.round(np.logspace(np.log10(20), np.log10(18000), 7),
                           1).tolist()
        Qs = parse_list(args.qs, float)
        gains = parse_list(args.gains, float)

        print(f"Fs={args.fs:g} Hz  threshold={args.threshold} dB  "
              f"n_freqs/config={args.n_freqs}")
        print(f"Types ({len(types)}): {types}")
        print(f"fcs ({len(fcs)}): {fcs}")
        print(f"Qs ({len(Qs)}): {Qs}")
        print(f"Gains ({len(gains)}): {gains}")
        print()

        failures = run_sweep(user_mod, types, fcs, Qs, gains,
                             args.fs, args.threshold, args.n_freqs,
                             args.impulse_length)

    if failures and args.plot_failures:
        for fail in failures:
            plot_comparison(user_mod, fail['type'], fail['fc'], fail['Q'],
                            fail['gain_db'], args.fs)

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
