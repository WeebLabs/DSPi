"""Host checks for the stereo spatializer (firmware/DSPi/spatial.c), the list in spec section 9.
Builds spatial.c as float (RP2350), Q28 as on RP2040, and Q28 held in int64 (no wrap, exact multiply).
Needs only numpy. Usage: python3 tools/spatial_test/spatial_check.py
"""
import ctypes
import os
import shutil
import subprocess
import sys
import tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.abspath(os.path.join(HERE, '..', '..', 'firmware', 'DSPi'))
DP = ctypes.POINTER(ctypes.c_double)
FP = ctypes.POINTER(ctypes.c_float)
DEF = [125.0, 150.0, 700.0, 10.0, 3.0, 100.0, 0.0]     # width, bass, shelf, span, atten, mid, trim


def build(variant):
    d = tempfile.mkdtemp(prefix='spatial_h_')
    for f in os.listdir(os.path.join(HERE, 'stubs')):
        shutil.copy(os.path.join(HERE, 'stubs', f), d)
    for f in ('spatial.c', 'spatial.h', 'bulk_params.h'):
        shutil.copy(os.path.join(FW, f), d)
    shutil.copy(os.path.join(HERE, 'harness.c'), d)
    so = os.path.join(d, 'lib.so')
    defs = {'float': ['-DPICO_RP2350=1'], 'q28': ['-DPICO_RP2350=0'],
            'q28wide': ['-DPICO_RP2350=0', '-DSP_Q28_WIDE']}[variant]
    subprocess.check_call(['cc', '-O2', '-ffp-contract=off', '-Wno-unknown-attributes', '-shared', '-fPIC',
                           '-I', d, os.path.join(d, 'harness.c'), '-o', so, '-lm'] + defs)
    lib = ctypes.CDLL(so)
    lib.h_config.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, FP, ctypes.c_float]
    lib.h_packet.argtypes = [ctypes.c_int, DP, DP, ctypes.c_int]
    lib.h_coeffs.argtypes = [DP]
    lib.h_enable_output.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.h_raw.argtypes = [ctypes.c_uint32]
    lib.h_codec.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    lib.h_process.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, DP, ctypes.c_int]
    lib.name = variant
    return lib


def config(lib, enabled=1, mode=0, mask=1, f=DEF, fs=48000.0):
    arr = (ctypes.c_float * 7)(*f)
    lib.h_config(enabled, mode, mask, arr, fs)


def coeffs(lib):
    o = (ctypes.c_double * 10)()
    lib.h_coeffs(o)
    return dict(zip('g gm a5 ah d fade w1 dwb aw t'.split(), list(o)))


def run(lib, L, R, blk=48, events=None, pair=0):
    """Packets of blk samples; events maps a sample index (a packet start) to a callable."""
    L = np.array(L, np.float64)
    R = np.array(R, np.float64)
    for s in range(0, len(L), blk):
        if events and s in events:
            events[s]()
        n = min(blk, len(L) - s)
        lb, rb = np.ascontiguousarray(L[s:s + n]), np.ascontiguousarray(R[s:s + n])
        lib.h_packet(pair, lb.ctypes.data_as(DP), rb.ctypes.data_as(DP), n)
        L[s:s + n], R[s:s + n] = lb, rb
    return L, R


def quant(lib, x):
    """The input as the kernel sees it (float32, or Q28)."""
    x = np.asarray(x, np.float64)
    return x.astype(np.float32).astype(np.float64) if lib.name == 'float' else np.round(x * 2 ** 28) / 2 ** 28


def onepole(x, a):
    y, out = 0.0, np.empty_like(x)
    for i, v in enumerate(x):
        y += a * (v - y)
        out[i] = y
    return out


def xtalk(y, c, g):
    """The kernel's crosstalk operator g*B*z^-D on a whole signal, in float64."""
    d = c['d']
    di, fr = int(d), d - int(d)
    tap = np.zeros_like(y)
    tap[di:] += (1 - fr) * y[:len(y) - di]
    tap[di + 1:] += fr * y[:len(y) - di - 1]
    lp = onepole(g * tap, c['a5'])
    return lp - onepole(lp, c['ah'])


FAILS = []


def check(ok, what):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', what))
    if not ok:
        FAILS.append(what)


def tones(n, fs, freqs, amps, seed=0):
    t = np.arange(n) / fs
    rng = np.random.default_rng(seed)
    return sum(a * np.sin(2 * np.pi * f * t + rng.uniform(0, 6.28)) for f, a in zip(freqs, amps))


def hf_peak(x, fs, cut=5000.0):
    """Peak above `cut` Hz of a Hann-windowed segment: steps are broadband, tones below cut are not."""
    w = np.hanning(len(x))
    sp = np.fft.rfft(np.asarray(x) * w)
    sp[np.fft.rfftfreq(len(x), 1.0 / fs) < cut] = 0
    return float(np.max(np.abs(np.fft.irfft(sp, len(x)))))


def extra_checks(lib, L, R):
    """Ramps, mid-path tail, the multi-pair dispatcher with live mask edits, rate reset, decay."""
    tol = 1e-9 if lib.name == 'float' else 0.0
    ftol = 1e-6 if lib.name == 'float' else 2e-7      # Q28: the mid tail stops at packet granularity
    fs, n = 48000.0, 9600
    base = [125, 150, 700, 10, 3, 100, 0]

    # Gain ramps: per sample, fade_len long, the same at any packet size.
    for idx, v, mode in ((6, -6.0, 0), (0, 180.0, 0), (5, 0.0, 1), (4, 12.0, 1)):
        outs = []
        for blk in (48, 37, 192):
            lib.h_init()
            config(lib, mode=mode, f=base)
            f2 = list(base); f2[idx] = v
            ev = {0: lambda: None, 7104: lambda: config(lib, mode=mode, f=f2)}   # lcm(48, 37, 192)
            outs.append(np.concatenate(run(lib, L, R, blk=blk, events=ev)))
        d = max(np.max(np.abs(outs[0] - outs[k])) for k in (1, 2))
        check(d <= ftol, '%s: param %d ramp identical at 48-, 37- and 192-sample packets (%.1e)' % (lib.name, idx, d))
    lib.h_init()
    config(lib, mode=0, f=base)
    dc = np.full(n, 0.5)
    ev = {0: lambda: None, 1776: lambda: config(lib, mode=0, f=base[:6] + [-6.0])}
    lo, _ro = run(lib, dc, dc, blk=37, events=ev)
    fl = int(coeffs(lib)['fade'])
    T = 10 ** (-6 / 20)
    want = 0.5 * (1 + (T - 1) * np.arange(1, fl + 1) / fl)
    seg = lo[1776:1776 + fl]
    check(np.max(np.abs(seg - want)) < 1e-4 and abs(lo[1776 + fl + 5] - 0.5 * T) < 1e-6,
          '%s: trim ramps linearly over exactly %d samples and lands on target' % (lib.name, fl))

    # Every live change, at 12 event phases: HF splatter at most an ideal 10 ms fade of the tones.
    tl = 0.25 * np.sin(2 * np.pi * 997 * np.arange(n) / fs)
    tr = 0.25 * np.sin(2 * np.pi * 1499 * np.arange(n) / fs)
    ev0 = 4800
    g = np.ones(n)
    g[ev0:ev0 + fl] = np.linspace(1, 0, fl)
    g[ev0 + fl:] = 0
    ref = hf_peak((tl * g)[ev0 - 2400:ev0 + 2400], fs)
    changes = [('enable', lambda f: (0, 0, f)), ('mode', lambda f: (1, 1, f)), ('mask', None),
               ('width', lambda f: (1, 0, f[:0] + [200] + f[1:])), ('corner', lambda f: (1, 0, f[:2] + [100] + f[3:])),
               ('trim', lambda f: (1, 0, f[:6] + [-6.0])), ('span', lambda f: (1, 1, f[:3] + [30] + f[4:])),
               ('atten', lambda f: (1, 1, f[:4] + [12] + f[5:])), ('mid', lambda f: (1, 1, f[:5] + [0] + f[6:]))]
    worst = {}
    for name, ch in changes:
        start_mode = 1 if name in ('span', 'atten', 'mid') else 0
        for off in range(0, 48 * 12, 48):
            lib.h_init()
            config(lib, mode=start_mode, f=base)
            if ch is None:
                ev = {0: lambda: None, ev0 + off: lambda: config(lib, mode=0, mask=0, f=base)}
            else:
                en, md, f2 = ch(list(base))
                ev = {0: lambda: None, ev0 + off: (lambda en=en, md=md, f2=f2: config(lib, enabled=en, mode=md, f=f2))}
            lo, ro = run(lib, tl, tr, events=ev)
            pk = max(hf_peak(x[ev0 - 2400:ev0 + 2400], fs) for x in (lo, ro))
            worst[name] = max(worst.get(name, 0.0), pk)
    bad = [k for k, v in worst.items() if v > 1.41 * ref]
    check(not bad, '%s: every live change at 12 event phases splatters no more than a 10 ms fade (+3 dB): %s'
          % (lib.name, ', '.join('%s %+.1f dB' % (k, 20 * np.log10(v / ref)) for k, v in worst.items())))

    # Mid path: gm settling at 0 lets the filter tail decay, then mono passes exactly.
    lib.h_init()
    config(lib, mode=1, f=base)
    mono = tones(n, fs, [300, 900], [0.3, 0.2], 3)
    ev = {0: lambda: None, 1776: lambda: config(lib, mode=1, f=base[:5] + [0.0, 0.0])}
    lo, ro = run(lib, mono, mono, events=ev)
    q = quant(lib, mono)
    lib.h_init()
    config(lib, mode=1, f=base)
    full = run(lib, mono, mono)[0]
    g = np.clip((np.arange(n) - 1776) / fl, 0, 1)
    ideal = [hf_peak((full * (1 - g) + q * g)[776:1776 + 4 * fl], fs),
             hf_peak((q * (1 - g) + full * g)[776:1776 + 4 * fl], fs)]
    lib.h_init()
    config(lib, mode=1, f=base)
    ev = {0: lambda: None, 1776: lambda: config(lib, mode=1, f=base[:5] + [0.0, 0.0])}
    lo, ro = run(lib, mono, mono, events=ev)
    tail = np.max(np.abs(lo[1776 + 2 * fl:1776 + 3 * fl] - q[1776 + 2 * fl:1776 + 3 * fl]))
    late = np.max(np.abs(lo[1776 + 4 * fl:] - q[1776 + 4 * fl:]))
    hf = hf_peak(lo[776:1776 + 4 * fl], fs)
    check(tail < 1e-5 and late <= tol and hf < 1.5 * ideal[0],
          '%s: mid to 0 %% decays (%.1e one fade later), then exact (%.1e), HF peak %.1e (10 ms crossfade %.1e)'
          % (lib.name, tail, late, hf, ideal[0]))
    lib.h_init()
    config(lib, mode=1, f=base[:5] + [0.0, 0.0])
    ev = {0: lambda: None, 1776: lambda: config(lib, mode=1, f=base)}
    lo, ro = run(lib, mono, mono, events=ev)
    hf = hf_peak(lo[776:1776 + 4 * fl], fs)
    check(hf < 1.5 * ideal[1], '%s: mid back to 100 %% ramps in without a step (HF peak %.1e, 10 ms crossfade %.1e)'
          % (lib.name, hf, ideal[1]))

    # Dispatcher over every pair with a live mask edit.
    P = lib.h_pairs()
    nn = 48
    sig = np.stack([tones(n, fs, [300 + 70 * k, 2100 - 90 * k], [0.3, 0.2], 10 + k) for k in range(2 * P)])
    lib.h_init()
    config(lib, mode=1, mask=(1 << P) - 1, f=base)
    out = sig.copy()
    mask = 0b0101 if P == 4 else 0b11
    for s in range(0, n, nn):
        if s == 4800:
            mask &= ~1
        buf = np.ascontiguousarray(out[:, s:s + nn].reshape(-1))
        lib.h_process(0, P - 1, mask, buf.ctypes.data_as(DP), nn)
        out[:, s:s + nn] = buf.reshape(2 * P, nn)
    ok = True
    for p in range(P):
        qs = [quant(lib, sig[2 * p + k]) for k in (0, 1)]
        if p == 0:      # processed, then faded out by the live mask edit
            ok &= max(np.max(np.abs(out[k, 4800 + 2 * fl:] - qs[k][4800 + 2 * fl:])) for k in (0, 1)) <= tol
            ok &= max(np.max(np.abs(out[k, 1000:4800] - qs[k][1000:4800])) for k in (0, 1)) > 1e-3
        elif (mask >> p) & 1:
            ok &= max(np.max(np.abs(out[2 * p + k, 1000:] - qs[k][1000:])) for k in (0, 1)) > 1e-3
        else:
            ok &= max(np.max(np.abs(out[2 * p + k] - qs[k])) for k in (0, 1)) <= tol
    check(ok, '%s: dispatcher over %d pairs: masked pairs run, unmasked pass exactly, a cleared bit fades '
          'its pair to exact passthrough' % (lib.name, P))

    # Rate change resets the pair: output matches a fresh start at the new rate.
    outs = []
    for fresh in (False, True):
        lib.h_init()
        config(lib, mode=1, f=base)
        if not fresh:
            run(lib, L[:2400], R[:2400])
        config(lib, mode=1, f=base, fs=96000.0)
        outs.append(np.concatenate(run(lib, L, R)))
    check(np.max(np.abs(outs[0] - outs[1])) <= tol, '%s: a rate change resets the pair state' % lib.name)

    # Impulse decay at the strongest loop (2.5 dB, 30 degrees, 96 kHz).
    lib.h_init()
    config(lib, mode=1, f=[125, 150, 700, 30, 2.5, 100, 0], fs=96000.0)
    run(lib, np.ones(2000) * 0.0, np.zeros(2000))
    imp = np.zeros(16384); imp[0] = 1.0
    lo, ro = run(lib, imp, np.zeros_like(imp))
    late = max(np.max(np.abs(lo[8192:])), np.max(np.abs(ro[8192:])))
    check(late < 1e-6, '%s: impulse response decays below 1e-6 after 8192 samples (%.1e)' % (lib.name, late))


def main():
    libs = [build(v) for v in ('float', 'q28', 'q28wide')]
    fs = 48000.0
    n = 9600
    L = tones(n, fs, [300, 1100, 3700], [0.3, 0.2, 0.1], 1)
    R = tones(n, fs, [450, 1700, 5200], [0.3, 0.2, 0.1], 2)

    for lib in libs[:2]:
        print('[%s]' % lib.name)
        lib.h_init()
        tol = 2e-6

        # Passthrough: width 100/100 at trim 0, and a disabled stage, are bit-exact.
        config(lib, f=[100, 100, 700, 10, 3, 100, 0])
        lo, ro = run(lib, L, R)
        check(np.array_equal(lo, quant(lib, L)) and np.array_equal(ro, quant(lib, R)),
              'width 100/100, trim 0 dB: bit-exact passthrough')
        lib.h_init()
        config(lib, enabled=0)
        lo, ro = run(lib, L, R)
        check(np.array_equal(lo, quant(lib, L)) and np.array_equal(ro, quant(lib, R)),
              'stage disabled: bit-exact passthrough')

        # Width: mono sum untouched, gains match the model after the fade.
        lib.h_init()
        config(lib, f=[180, 60, 400, 10, 3, 100, 0])
        c = coeffs(lib)
        lo, ro = run(lib, L, R)
        ql, qr = quant(lib, L), quant(lib, R)
        dsum = np.max(np.abs((lo + ro) - (ql + qr)))
        check(dsum == 0 if lib.name != 'float' else dsum < 1e-6,
              'width: L+R unchanged (%s, max %.1e)' % ('exact' if lib.name != 'float' else 'float', dsum))
        S = 0.5 * (ql - qr)
        dS = c['w1'] * S + c['dwb'] * onepole(S, c['aw'])
        k = int(c['fade']) + 1
        err = max(np.max(np.abs(lo[k:] - (ql + dS)[k:])), np.max(np.abs(ro[k:] - (qr - dS)[k:])))
        check(err < tol, 'width: matches the float64 model after the fade (max error %.1e)' % err)
        lib.h_init()
        config(lib, f=[200, 200, 700, 10, 3, 100, 0])
        x = tones(4800, fs, [1000], [0.25])
        lo, ro = run(lib, x, -x)
        gain = 20 * np.log10(np.max(np.abs(lo[2400:])) / np.max(np.abs(x[2400:])))
        check(abs(gain - 6.02) < 0.05, 'width 200 %%: anti-phase content +%.2f dB (spec 5: +6.02)' % gain)

        # XTC: the recursion is the exact inverse of its crosstalk model (the acoustic null).
        for mid in (100.0, 0.0):
            lib.h_init()
            config(lib, mode=1, f=[100, 100, 700, 15, 2.5, mid, 0])
            c = coeffs(lib)
            lo, ro = run(lib, L, R)
            ql, qr = quant(lib, L), quant(lib, R)
            k = int(c['fade']) + 3000          # past the fade and the 250 Hz filter's tail of faded history
            Mp, Sp = 0.5 * (lo + ro), 0.5 * (lo - ro)
            eS = Sp - xtalk(Sp, c, c['g'])
            eM = Mp + xtalk(Mp, c, c['g'])
            es_err = np.max(np.abs((eS - 0.5 * (ql - qr))[k:]))
            em_err = np.max(np.abs((eM - 0.5 * (ql + qr))[k:]))
            if mid == 100.0:
                check(es_err < 1e-4 and em_err < 1e-4,
                      'XTC full: both ears get their own channel (side %.1e, mid %.1e)' % (es_err, em_err))
            else:
                check(es_err < 1e-4, 'XTC mid 0 %%: the side ear signal is exact (%.1e)' % es_err)
                dsum = np.max(np.abs((lo + ro) - (ql + qr)))
                check(dsum == 0 if lib.name != 'float' else dsum < 1e-6,
                      'XTC mid 0 %%: the mid (L+R) is untouched (max %.1e)' % dsum)

        # Delay coverage.
        lib.h_init()
        config(lib, mode=1, f=[100, 100, 700, 30, 3, 100, 0], fs=96000.0)
        d96 = coeffs(lib)['d']
        config(lib, mode=1, f=[100, 100, 700, 5, 3, 100, 0], fs=44100.0)
        d44 = coeffs(lib)['d']
        check(20.0 < d96 < 30.0 and abs(d44 - 1.0) < 1e-4,
              'delay: 30 deg at 96 kHz is %.2f samples (fits the ring), 5 deg at 44.1 kHz floors at %.2f' % (d96, d44))

        # Fades and skips.
        lib.h_init()
        config(lib, f=[180, 160, 700, 10, 3, 100, -3])
        ev = {4800: lambda: config(lib, enabled=0, f=[180, 160, 700, 10, 3, 100, -3])}
        lo, ro = run(lib, L, R, events=ev)
        k = 4800 + int(coeffs(lib)['fade']) + 1
        check(np.array_equal(lo[k:], quant(lib, L)[k:]), 'disable: after the 10 ms fade the output is the input exactly')
        lib.h_init()
        config(lib, f=[180, 160, 700, 10, 3, 100, 0])
        ev = {4800: lambda: lib.h_raw(3), 4848: lambda: lib.h_raw(0)}
        lo, ro = run(lib, L, R, events=ev)
        check(np.array_equal(lo[4800:4848], quant(lib, L)[4800:4848]), 'RAW on the pair: skipped at once (exact input)')
        check(abs(lo[4848] - quant(lib, L)[4848]) < 0.02, 'RAW cleared: the pair fades back in from zero')
        lib.h_init()
        config(lib, mode=1, f=[180, 160, 700, 10, 3, 100, 0])
        Rz = np.array(R)
        Rz[4800:] = 0.0                          # the matrix zeroes a disabled output
        ev = {4800: lambda: lib.h_enable_output(1, 0)}
        lo, ro = run(lib, L, Rz, events=ev)
        check(np.all(ro[4800:] == 0.0) and np.array_equal(lo[4800:], quant(lib, L)[4800:]),
              'output disabled on a running pair: its buffer stays exactly zero, the other passes')
        lib.h_enable_output(1, 1)
        lib.h_init()
        config(lib, mode=0, f=[180, 160, 700, 10, 3, 100, 0])
        ev = {2400: lambda: config(lib, mode=1, f=[180, 160, 700, 10, 3, 100, 0]),
              2448: lambda: config(lib, mode=0, f=[180, 160, 700, 10, 3, 100, 0]),
              2496: lambda: config(lib, mode=1, f=[180, 160, 700, 10, 3, 100, 0])}
        lo, ro = run(lib, L, R, events=ev)
        c = coeffs(lib)
        ql, qr = quant(lib, L), quant(lib, R)
        k = 6000
        Sp = 0.5 * (lo - ro)
        es_err = np.max(np.abs((Sp - xtalk(Sp, c, c['g']) - 0.5 * (ql - qr))[k:]))
        check(np.all(np.isfinite(lo)) and es_err < 1e-4,
              'three mode SETs inside one fade: settles on the last mode, no stranded state (%.1e)' % es_err)
        # The switch passes through exact passthrough and never overshoots either mode's own delta.
        lib.h_init()
        config(lib, mode=0, f=[180, 160, 700, 10, 3, 100, 0])
        wd = np.max(np.abs(run(lib, L, R)[0][2000:] - ql[2000:]))
        lib.h_init()
        config(lib, mode=1, f=[180, 160, 700, 10, 3, 100, 0])
        xd = np.max(np.abs(run(lib, L, R)[0][2000:] - ql[2000:]))
        lib.h_init()
        config(lib, mode=0, f=[180, 160, 700, 10, 3, 100, 0])
        lo, ro = run(lib, L, R, events={2400: lambda: config(lib, mode=1, f=[180, 160, 700, 10, 3, 100, 0])})
        seg = lo[2400:4800] - ql[2400:4800]
        check(np.any(seg == 0.0) and np.max(np.abs(seg)) <= 1.01 * max(wd, xd),
              'mode switch: fades through exact passthrough, peak delta %.3f within the modes\' %.3f' % (
                  np.max(np.abs(seg)), max(wd, xd)))

        # Packet-size independence of the delay slew and the fades (no gain change).
        outs = []
        for blk in (48, 37):
            lib.h_init()
            config(lib, mode=1, f=[100, 100, 700, 5, 3, 100, 0])
            ev = {0: lambda: None, 1776: lambda: config(lib, mode=1, f=[100, 100, 700, 30, 3, 100, 0])}
            outs.append(run(lib, L, R, blk=blk, events=ev)[0])
        check(np.array_equal(outs[0], outs[1]), 'span change: identical output at 48- and 37-sample packets')

    for lib in libs[:2]:
        extra_checks(lib, L, R)

    # Q28: no wrap and no multiply error, against the int64 build, which cannot wrap.
    print('[q28 range]')
    q, qx = libs[1], libs[2]
    worst = fworst = 0.0
    for fs in (44100.0, 48000.0, 96000.0):
        for span in (5.0, 30.0):
            for trim in (0.0, -6.0):
                for amp in (1.0, 2.0, 7.9):
                    for mode, mid in ((1, 100.0), (1, 0.0), (0, 100.0)):
                        f = [200, 200, 700, span, 2.5, mid, trim] if mode == 1 else [200, 0, 1500, span, 2.5, mid, trim]
                        nn = 4800
                        t = np.arange(nn) / fs
                        x = amp * np.sign(np.sin(2 * np.pi * 480 * t))         # square at the side peak
                        u = amp * np.sign(np.sin(2 * np.pi * 690 * t + 1.0))
                        for shape, y in (('antiphase', -x), ('in-phase', x), ('left only', 0 * x),
                                         ('uncorrelated', u)):
                            res = []
                            for lib in (q, qx):
                                lib.h_init()
                                config(lib, mode=mode, f=f, fs=fs)
                                res.append(run(lib, x, y))
                            dmax = max(np.max(np.abs(res[0][k] - res[1][k])) for k in (0, 1))
                            peak = max(np.max(np.abs(res[0][k])) for k in (0, 1))
                            # Float differs from Q28 only by the clamps, so it is a reference below them.
                            fdev = 0.0
                            if amp == 1.0:
                                xs, ys = 0.25 * x / amp, 0.25 * y / amp
                                qq = []
                                for lib in (q, libs[0]):
                                    lib.h_init()
                                    config(lib, mode=mode, f=f, fs=fs)
                                    qq.append(run(lib, xs, ys))
                                sk = int(0.03 * fs)        # past the fade and gain ramps, whose steps round
                                fdev = max(np.max(np.abs(qq[0][k][sk:] - qq[1][k][sk:])) for k in (0, 1))
                            worst = max(worst, dmax)
                            fworst = max(fworst, fdev)
                            if dmax > 1e-5 or peak > 7.9 + 1e-6 or fdev > 1e-5:
                                check(False, 'Q28 %s at fs %g span %g trim %g amp %g mode %d mid %g: diff %.1e '
                                      'float dev %.2e peak %.2f' % (shape, fs, span, trim, amp, mode, mid, dmax,
                                                                     fdev, peak))
    check(worst < 1e-5, 'Q28 against the int64 build (no wrap) over rates, spans, trims, four input shapes to +-7.9: '
          'worst %.1e' % worst)
    check(fworst < 1e-5, 'Q28 against the float build below its clamps, four input shapes: worst %.1e' % fworst)

    # Codec and notify offsets.
    print('[codec]')
    for lib in libs[:2]:
        lib.h_init()
        pkt = bytearray(32)
        pkt[0], pkt[1], pkt[2] = 1, 5, 0xFF
        import struct
        struct.pack_into('<7f', pkt, 4, float('nan'), 100, 700, 10, float('inf'), 100, 0)
        out = ctypes.create_string_buffer(32)
        lib.h_codec(bytes(pkt), out)
        o = out.raw
        wb, = struct.unpack_from('<f', o, 4)
        at, = struct.unpack_from('<f', o, 20)
        mask = (1 << lib.h_pairs()) - 1
        check(o[0] == 0 and o[1] == 0 and o[2] == mask and wb == 125.0 and at == 3.0,
              '%s: unknown mode loads off, mask masked to %#x, non-finite floats take the defaults' % (lib.name, mask))
        check(lib.h_notify_check() == 0, '%s: notify offsets match WireBulkParams' % lib.name)

    print('%d failing checks' % len(FAILS))
    sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
    main()
