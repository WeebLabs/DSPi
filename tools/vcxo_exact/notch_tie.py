#!/opt/homebrew/bin/python3.14
"""In-band TIE with the TDC's period-15-VCO artifact lines removed.

Usage: notch_tie.py LOG [LOG ...]
Bins within +-8 Hz of multiples of f_net/3 are dropped (f_net = net pulse rate =
ppm x 307.2 Hz, or 3076.9 Hz for the scheme-4 artifact test).
"""
import sys, importlib.util
import numpy as np
from scipy import signal

spec = importlib.util.spec_from_file_location("a", __file__.replace("notch_tie.py", "analyze_tie.py"))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)

for path in sys.argv[1:]:
    for c in a.parse(path):
        h = c["hdr"]
        if not c["L"] or h["scheme"] not in ("2", "4"):
            continue
        L = np.cumsum(np.array(c["L"], float))[16:] / 256 * a.VCO
        x, _ = a.detrend(L, 75e3)
        f, p = signal.welch(x, fs=75e3, nperseg=3750, window="hann")
        p = p / np.maximum(a.cic_h(f) ** 2, 1e-12)
        fnet = 400e3 / 130 if h["scheme"] == "4" else abs(float(h["ppm"])) * 307.2
        keep = np.ones_like(f, bool)
        if fnet > 0:
            for m in range(1, int(20e3 / (fnet / 3)) + 2):
                keep &= np.abs(f - m * fnet / 3) > 30
        m = (f >= 20) & (f <= 20e3)
        full = np.sqrt(np.trapezoid(p[m], f[m])) * 1e9
        # Removed bins are replaced by the median of the kept in-band bins.
        q = p.copy()
        q[m & ~keep] = np.median(p[m & keep])
        notched = np.sqrt(np.trapezoid(q[m], f[m])) * 1e9
        print(f"{path.split('/')[-1]},scheme={h['scheme']},ppm={h['ppm']},load={h['load']},dither={h['dither']},"
              f"sp={h.get('sp', '6')},inband={full:.4f},inband_artifact_lines_removed={notched:.4f}")
