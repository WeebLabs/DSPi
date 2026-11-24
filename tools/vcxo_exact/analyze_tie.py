#!/opt/homebrew/bin/python3.14
"""TIE analysis of recorder dumps (RD lines) from the vcxo_exact bench.

Usage: analyze_tie.py LOG [LOG ...]
L stream: phase in VCO cycles x 256 at 75 kHz after a CIC5/8 (droop is undone here).
S stream: phase in VCO cycles x 256 at 600 kHz. One VCO cycle = 1/1.536 GHz.
Prints one CSV row per JIT case: rms TIE per band in ns, largest in-band spur, fitted trim.
"""
import sys
import numpy as np
from scipy import signal

VCO = 1.0 / 1.536e9


def cic_h(f, fs=600e3, r=8, n=5):
    x = np.pi * np.asarray(f) / fs
    with np.errstate(invalid="ignore", divide="ignore"):
        h = np.where(x == 0, 1.0, np.sin(r * x) / (r * np.sin(x)))
    return np.abs(h) ** n


def band_rms(f, p, lo, hi):
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return float("nan")
    return float(np.sqrt(np.trapezoid(p[m], f[m])))


def detrend(x, fs):
    t = np.arange(len(x)) / fs
    a, b = np.polyfit(t, x, 1)
    return x - (a * t + b), a


def parse(path):
    cases, cur = [], None
    for line in open(path):
        p = line.strip().split(",")
        if p[0] == "JIT":
            cur = {"hdr": dict(kv.split("=") for kv in p[1:]), "L": [], "S": [], "rec": []}
            cases.append(cur)
        elif p[0] == "REC" and cur is not None:
            cur["rec"].append(dict(kv.split("=") for kv in p[1:]))
        elif p[0] == "RD" and cur is not None:
            cur[p[1]].extend(int(v) for v in p[3:])
    return cases


def analyse(c):
    out = dict(c["hdr"])
    L = np.cumsum(np.array(c["L"], dtype=np.float64))[16:] / 256.0 * VCO
    S = np.cumsum(np.array(c["S"], dtype=np.float64))[8:] / 256.0 * VCO
    fsL, fsS = 75e3, 600e3
    xL, slope = detrend(L, fsL)
    out["fit_ppm"] = slope * 1e6
    f, p = signal.welch(xL, fs=fsL, nperseg=3750, window="hann", detrend="linear")
    p = p / np.maximum(cic_h(f) ** 2, 1e-12)
    for lo, hi in ((20, 20e3), (200, 20e3), (1e3, 20e3)):
        out[f"rms_{int(lo)}_20k_ns"] = band_rms(f, p, lo, hi) * 1e9
    m = (f >= 20) & (f <= 20e3)
    df = f[1] - f[0]
    k = np.argmax(p * m)
    spur = np.sqrt(np.sum(p[max(k - 2, 0):k + 3]) * df)
    out["spur_hz"] = f[k]
    out["spur_ns"] = spur * 1e9
    xS, _ = detrend(S, fsS)
    f2, p2 = signal.welch(xS, fs=fsS, nperseg=6000, window="hann", detrend="linear")
    out["rms_20k_300k_ns"] = band_rms(f2, p2, 20e3, 300e3) * 1e9
    out["rms_20_20k_fullrate_ns"] = band_rms(f2, p2, 100, 20e3) * 1e9
    out["rms_total_fullrate_ns"] = float(np.std(xS)) * 1e9
    out["rec"] = ";".join(f"m{r['mode']}:skips={r['skips']},lost={r['lost']},late={r['filler_late']}"
                          for r in c["rec"])
    return out


if __name__ == "__main__":
    keys = None
    for path in sys.argv[1:]:
        for c in parse(path):
            if not c["L"] or not c["S"]:
                continue
            r = analyse(c)
            if keys is None:
                keys = list(r.keys())
                print(",".join(keys))
            print(",".join(f"{r[k]:.4g}" if isinstance(r[k], float) else str(r[k]) for k in keys))
