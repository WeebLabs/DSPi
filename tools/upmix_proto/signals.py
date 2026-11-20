"""Deterministic synthetic music-like test material for the upmix prototype.

Every test is a dict: name -> stem, where a stem is (L, R, kind, onsets).
kind is 'direct', 'ambience' or 'antiphase'; onsets are sample indices or None.
"""
import numpy as np
from scipy import signal

FS = 48000
DUR = 6.0
N = int(FS * DUR)


def rms(x):
    return float(np.sqrt(np.mean(x * x)) + 1e-30)


def norm(x, db):
    return x * (10 ** (db / 20.0) / rms(x))


def bandnoise(seed, lo, hi, n=N):
    x = np.random.default_rng(seed).standard_normal(n)
    sos = signal.butter(4, [lo, hi], 'bandpass', fs=FS, output='sos')
    return signal.sosfilt(sos, x)


def am_env(seed, rate, depth, n=N):
    """Slow random amplitude contour, phrase-like rather than periodic."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / FS
    e = np.ones(n)
    for k in range(3):
        f = rate * (0.6 + 0.8 * rng.random())
        e += depth / 3 * np.sin(2 * np.pi * f * t + rng.random() * 6.28)
    return np.clip(e, 0.05, None)


def voice(seed):
    """Syllabic band noise plus a vibrato harmonic series, 150 Hz to 4 kHz."""
    rng = np.random.default_rng(seed)
    t = np.arange(N) / FS
    f0 = 165 * (1 + 0.01 * np.sin(2 * np.pi * 5.5 * t)) * (1 + 0.1 * np.sin(2 * np.pi * 0.3 * t))
    ph = 2 * np.pi * np.cumsum(f0) / FS
    tone = sum(np.sin(k * ph + rng.random() * 6) / k for k in range(1, 16))
    tone = signal.sosfilt(signal.butter(2, [150, 4000], 'bandpass', fs=FS, output='sos'), tone)
    syl = np.clip(np.sin(2 * np.pi * 4.0 * t + 0.3 * np.sin(2 * np.pi * 0.7 * t)), 0, None) ** 0.7
    x = norm(tone, 0) + 0.4 * norm(bandnoise(seed + 1, 300, 3500), 0)
    return x * syl * am_env(seed + 2, 0.4, 0.5)


def plucks(seed, period=0.25, f_lo=110, f_hi=440, decay=0.35, lo=150, hi=5000):
    """Harmonic plucked notes with exponential decay; returns signal and onsets."""
    rng = np.random.default_rng(seed)
    x = np.zeros(N)
    onsets = []
    step = int(period * FS)
    for s in range(int(0.05 * FS), N - step, step):
        f = f_lo * (f_hi / f_lo) ** rng.random()
        L = min(int(4 * decay * FS), N - s)
        t = np.arange(L) / FS
        note = sum(np.sin(2 * np.pi * f * k * t + rng.random() * 6) * np.exp(-t * k / decay) / k
                   for k in range(1, 12) if f * k < 12000)
        x[s:s + L] += note * (0.7 + 0.3 * rng.random())
        onsets.append(s)
    x = signal.sosfilt(signal.butter(2, [lo, hi], 'bandpass', fs=FS, output='sos'), x)
    return x, np.array(onsets)


def hits(seed, period, lo, hi, decay, accent=True):
    """Noise-burst percussion (hats, snare, claps) with fast attack."""
    rng = np.random.default_rng(seed)
    x = np.zeros(N)
    onsets = []
    step = int(period * FS)
    for j, s in enumerate(range(int(0.1 * FS), N - step, step)):
        L = min(int(6 * decay * FS), N - s)
        t = np.arange(L) / FS
        a = (1.0 if (j % 2 == 0 or not accent) else 0.6) * (0.8 + 0.2 * rng.random())
        x[s:s + L] += rng.standard_normal(L) * np.exp(-t / decay) * a
        onsets.append(s)
    x = signal.sosfilt(signal.butter(2, [lo, hi], 'bandpass', fs=FS, output='sos'), x)
    return x, np.array(onsets)


def kick_bass(seed):
    rng = np.random.default_rng(seed)
    x = np.zeros(N)
    for s in range(0, N - FS // 2, FS // 2):
        L = min(int(0.4 * FS), N - s)
        t = np.arange(L) / FS
        f = 50 + 70 * np.exp(-t / 0.03)
        x[s:s + L] += np.sin(2 * np.pi * np.cumsum(f) / FS) * np.exp(-t / 0.12)
    t = np.arange(N) / FS
    notes = 55 * 2 ** (rng.integers(0, 7, size=int(DUR * 2) + 1) / 12)
    fb = np.repeat(notes, FS // 2)[:N]
    ph = 2 * np.pi * np.cumsum(fb) / FS
    bass = (np.sin(ph) + 0.3 * np.sin(2 * ph) + 0.15 * np.sin(3 * ph)) * am_env(seed + 1, 2.0, 0.6)
    return norm(x, 0) + 0.8 * norm(bass, 0)


def reverb(x, seed, rt60=1.4, length=1.5, predelay=0.02):
    """Two independent exponentially decaying noise IRs: a diffuse late tail."""
    rng = np.random.default_rng(seed)
    n = int(length * FS)
    t = np.arange(n) / FS
    env = np.exp(-6.91 * t / rt60)
    pd = int(predelay * FS)
    out = []
    for ch in range(2):
        ir = np.zeros(n + pd)
        ir[pd:] = rng.standard_normal(n) * env
        ir = signal.sosfilt(signal.butter(1, 6000, 'low', fs=FS, output='sos'), ir)
        out.append(signal.fftconvolve(x, ir)[:N])
    return out[0], out[1]


def pan(x, p):
    """Constant-power pan; p = 0 centre, 1 hard left."""
    th = np.pi / 4 * (1 + p)
    return x * np.sin(th), x * np.cos(th)


def instrument(seed):
    """Band-limited, phrase-modulated noise plus partials: a sustained pad/keys stem."""
    x = norm(bandnoise(seed, 250, 5000), 0) * am_env(seed + 5, 1.5, 0.8)
    p, _ = plucks(seed + 9, period=0.5, decay=0.5)
    return x + 0.7 * norm(p, 0)


def amb_noise(seed):
    sl = bandnoise(seed, 150, 9000) * am_env(seed + 3, 0.2, 0.3)
    sr = bandnoise(seed + 100, 150, 9000) * am_env(seed + 4, 0.2, 0.3)
    return sl, sr


def build_tests():
    T = {}
    src = norm(instrument(10), -12)
    for p in (0.0, 0.25, 0.5, 0.75, 0.9, 1.0):
        l, r = pan(src, p)
        T['pan%03d' % int(p * 100)] = {'src': (l, r, 'direct', None)}

    g, on = plucks(20)
    g = norm(g, -12)
    T['hardL'] = {'gtrL': (g, 0 * g, 'direct', on)}

    g2, on2 = plucks(21, period=0.25 * 1.07)
    g2 = norm(g2, -12)
    T['uncorrLR'] = {'gtrL': (g, 0 * g, 'direct', on), 'gtrR': (0 * g2, g2, 'direct', on2)}

    al, ar = amb_noise(30)
    tl, tr = reverb(voice(31), 32)
    a = rms(np.concatenate([al, ar])); b = rms(np.concatenate([tl, tr]))
    T['diffuse'] = {'amb': (norm(al, -18) * 1, norm(ar, -18), 'ambience', None),
                    'tail': (tl / b * 10 ** (-18 / 20), tr / b * 10 ** (-18 / 20), 'ambience', None)}

    s = norm(instrument(40), -12)
    T['antiphase'] = {'ap': (s, -s, 'antiphase', None)}

    v = norm(voice(50), -12)
    gl, gon = plucks(51)
    gl = norm(gl, -15)
    rl, rr = reverb(v + gl, 52)
    k = 10 ** (-12 / 20) / rms(np.concatenate([rl, rr]))   # wet about 12 dB under the voice
    vl, vr = pan(v, 0)
    T['mix_vox_gtr_amb'] = {'vox': (vl, vr, 'direct', None),
                            'gtrL': (gl, 0 * gl, 'direct', gon),
                            'amb': (rl * k, rr * k, 'ambience', None)}

    kb = norm(kick_bass(60), -9)
    keys = norm(instrument(61), -15)
    al, ar = amb_noise(62)
    T['mix_kick_keysL'] = {'kick': pan(kb, 0) + ('direct', None),
                           'keysL': (keys, 0 * keys, 'direct', None),
                           'amb': (norm(al, -27), norm(ar, -27), 'ambience', None)}

    v = norm(voice(70), -12)
    hh, hon = hits(71, 0.125, 4000, 11000, 0.025)
    hh = norm(hh, -18)
    al, ar = amb_noise(72)
    T['mix_vox_hatsL'] = {'vox': pan(v, 0) + ('direct', None),
                          'hatsL': (hh, 0 * hh, 'direct', hon),
                          'amb': (norm(al, -27), norm(ar, -27), 'ambience', None)}

    sn, son = hits(80, 0.5, 300, 6000, 0.06, accent=False)
    sn = norm(sn, -15)
    tl, tr = reverb(sn + norm(voice(81), -18), 82)
    k = 10 ** (-18 / 20) / rms(np.concatenate([tl, tr]))
    al, ar = amb_noise(83)
    sl, sr = pan(sn, 0.8)
    T['mix_snareL_amb'] = {'snareL': (sl, sr, 'direct', son),
                           'amb': (tl * k + norm(al, -27), tr * k + norm(ar, -27), 'ambience', None)}
    return T
