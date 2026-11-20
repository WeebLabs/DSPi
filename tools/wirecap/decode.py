"""Wire decoders for captured pin lines: S/PDIF biphase mark and standard I2S,
plus the cross-line alignment measurement. Times are in sampler ticks unless named _ns.
"""

from __future__ import annotations

import numpy as np

# Channel status byte 3 low nibble per IEC 60958-3 (bit 24 is the LSB).
CS_RATE_CODE = {44100: 0x0, 48000: 0x2, 32000: 0x3, 88200: 0x8, 96000: 0xA, 176400: 0xC, 192000: 0xE}
PREAMBLES = {(3, 3, 1, 1): "X", (3, 2, 1, 2): "Y", (3, 1, 1, 3): "Z"}


def _edges(line: np.ndarray):
    """Indices of the first sample after each transition, and the level it starts."""
    tr = np.flatnonzero(np.diff(line.astype(np.int8))) + 1
    return tr, line[tr] if len(tr) else np.zeros(0, np.uint8)


def spdif_decode(line: np.ndarray, fs: float, rate: int) -> dict:
    """Decode one S/PDIF line. fs is the sampler rate in Hz, rate the audio rate."""
    T = fs / (128.0 * rate)                       # one half cell, in ticks
    tr, lev = _edges(line)
    out = dict(kind="spdif", rate=rate, half_cell_ticks=T, errors=[])
    if len(tr) < 200:
        out["errors"].append(f"only {len(tr)} transitions")
        out["frames"] = []
        return out
    runs = np.diff(tr).astype(np.float64)
    n = np.rint(runs / T).astype(np.int64)
    bad = (n < 1) | (n > 3) | (np.abs(runs / T - n) > 0.4)
    stats = {}
    for k in (1, 2, 3):
        r = runs[(n == k) & ~bad] / fs * 1e9
        if len(r):
            stats[f"{k}T"] = dict(count=int(len(r)), mean_ns=float(r.mean()), std_ns=float(r.std()),
                                  min_ns=float(r.min()), max_ns=float(r.max()),
                                  nominal_ns=k * T / fs * 1e9)
    out["cell_timing"] = stats
    out["bad_runs"] = int(bad.sum())

    # Half-cell level sequence and the half-cell position where each run starts.
    nn = np.where(bad, 0, n)
    hpos = np.concatenate([[0], np.cumsum(nn)])
    hc = np.repeat(lev[:-1], nn)
    # A preamble is the only place a 3T run occurs; the run tuple tells a start from Z's trailing 3T.
    cand = np.flatnonzero((n[:-3] == 3) & ~bad[:-3])
    starts = [s for s in cand if tuple(int(x) for x in n[s:s + 4]) in PREAMBLES]
    subs = []
    for s in starts:
        if s + 3 >= len(n):
            break
        h = hpos[s]
        if h + 64 > len(hc):
            break
        cells = hc[h:h + 64]
        kind = PREAMBLES.get(tuple(int(x) for x in n[s:s + 4]))
        bits = (cells[8::2] != cells[9::2]).astype(np.int64)        # slots 4..31
        bmc_ok = bool(np.all(cells[8::2] != np.concatenate([[cells[7]], cells[9:63:2]])))
        subs.append((int(tr[s]), int(h), kind, bits, bmc_ok))
    if not subs:
        out["errors"].append("no preambles")
        out["frames"] = []
        return out

    t = np.array([x[0] for x in subs])
    hs = np.array([x[1] for x in subs])
    kinds = [x[2] for x in subs]
    bits = np.array([x[3] for x in subs])
    framing_ok = np.diff(hs) == 64
    out["subframes"] = len(subs)
    out["framing_breaks"] = int((~framing_ok).sum())
    out["unknown_preambles"] = int(sum(k is None for k in kinds))
    out["bmc_violations"] = int(sum(not x[4] for x in subs))
    weights = 1 << np.arange(24, dtype=np.int64)
    raw = bits[:, :24] @ weights
    samples = np.where(raw >= 1 << 23, raw - (1 << 24), raw)
    v, u, c, p = bits[:, 24], bits[:, 25], bits[:, 26], bits[:, 27]
    parity_bad = (bits.sum(axis=1) & 1) != 0
    out["parity_errors"] = int(parity_bad.sum())
    out["v_set"] = int(v.sum())
    out["u_set"] = int(u.sum())

    # Frames: a left subframe (X or Z) followed directly by Y.
    frames = []
    for i in range(len(subs) - 1):
        if kinds[i] in ("X", "Z") and kinds[i + 1] == "Y" and hs[i + 1] - hs[i] == 64:
            frames.append((t[i], int(samples[i]), int(samples[i + 1]), kinds[i] == "Z",
                           int(c[i]), int(c[i + 1])))
    out["frames"] = frames
    order_bad = sum(1 for i in range(len(subs) - 1)
                    if (kinds[i] == "Y") == (kinds[i + 1] == "Y"))
    out["xy_order_errors"] = order_bad

    zi = [i for i, f in enumerate(frames) if f[3]]
    out["z_count"] = len(zi)
    out["z_spacing"] = sorted(set(int(b - a) for a, b in zip(zi, zi[1:])))
    # Gaps in the frame list would make Z spacing meaningless; frames must be contiguous.
    ft = np.array([f[0] for f in frames])
    if len(ft) > 2:
        per = np.median(np.diff(ft))
        out["frame_gaps"] = int((np.abs(np.diff(ft) - per) > per * 0.25).sum())
    if zi and zi[0] + 192 <= len(frames):
        z0 = zi[0]
        for ch, col in (("a", 4), ("b", 5)):
            cbits = [frames[z0 + k][col] for k in range(192)]
            out[f"cs_{ch}"] = bytes(sum(cbits[8 * b + i] << i for i in range(8)) for b in range(24)).hex()
        cs = bytes.fromhex(out["cs_a"])
        out["cs_rate_code"] = cs[3] & 0x0F
        out["cs_rate_ok"] = CS_RATE_CODE.get(rate) == (cs[3] & 0x0F)
    return out


def i2s_decode(bck: np.ndarray, lrck: np.ndarray, data: np.ndarray, fs: float) -> dict:
    out = dict(kind="i2s", errors=[])
    rise = np.flatnonzero(np.diff(bck.astype(np.int8)) == 1) + 1
    fall = np.flatnonzero(np.diff(bck.astype(np.int8)) == -1) + 1
    lr_edges, lr_lev = _edges(lrck)
    if len(rise) < 200 or len(lr_edges) < 4:
        out["errors"].append(f"too few clock edges (bck {len(rise)}, lrck {len(lr_edges)})")
        out["frames"] = []
        return out
    bck_per = np.diff(rise)
    out["bck_period_ticks"] = dict(mean=float(bck_per.mean()), min=int(bck_per.min()), max=int(bck_per.max()))
    out["bck_hz"] = float(fs / bck_per.mean())

    def nearest(src, ref):
        # Edges outside the span of ref have no neighbour on one side; skip them.
        src = src[(src > ref[0]) & (src < ref[-1])]
        if not len(src):
            return np.zeros(1, np.int64)
        j = np.searchsorted(ref, src)
        return np.minimum(np.abs(src - ref[j - 1]), np.abs(src - ref[j]))

    out["lrck_to_bck_fall_max_ticks"] = int(nearest(lr_edges, fall).max())
    d_edges, _ = _edges(data)
    out["data_to_bck_fall_max_ticks"] = int(nearest(d_edges, fall).max()) if len(d_edges) else 0

    lf = lr_edges[lr_lev == 0]                   # LRCLK falling: left word starts
    # BCK rising edges between consecutive left starts must number exactly 64.
    idx = np.searchsorted(rise, lf)
    per_frame = np.diff(idx)
    out["bck_per_frame"] = sorted(set(int(x) for x in per_frame))
    lr_r = lr_edges[lr_lev == 1]
    halves = np.searchsorted(rise, lr_r) - np.searchsorted(rise, lf[np.searchsorted(lf, lr_r) - 1]) \
        if len(lr_r) and len(lf) else np.zeros(0)
    out["bck_per_left_half"] = sorted(set(int(x) for x in halves[np.searchsorted(lf, lr_r) > 0]))
    dbits = data[np.minimum(rise, len(data) - 1)].astype(np.int64)
    frames = []
    pad_bad = 0
    for k in range(len(lf) - 1):
        i0 = idx[k]
        if idx[k + 1] - i0 != 64 or i0 + 65 > len(dbits):
            continue
        words = []
        for h in (0, 1):
            b = dbits[i0 + 1 + 32 * h: i0 + 33 + 32 * h]       # MSB first
            w = int("".join(str(x) for x in b), 2)
            if w & 0xFF:
                pad_bad += 1
            s = w >> 8
            words.append(s - (1 << 24) if s >= 1 << 23 else s)
        frames.append((int(lf[k]), words[0], words[1], False, 0, 0))
    out["low8_nonzero_words"] = pad_bad   # informational: the firmware sends 32 significant bits
    out["frames"] = frames
    return out


def frame_period_ticks(fs: float, rate: int) -> float:
    return fs / rate


def match_offset(a: dict, b: dict, fs: float, rate: int) -> dict | None:
    """Time from a sample on line a to the same sample on line b, by exact value match."""
    fa, fb = a.get("frames", []), b.get("frames", [])
    if len(fa) < 6 or len(fb) < 6:
        return None
    va = np.array([f[1] for f in fa]); vb = np.array([f[1] for f in fb])
    if len(set(va.tolist())) < 0.6 * len(va):
        return dict(error="signal not unique enough to match (is the generator running?)")
    ta = np.array([f[0] for f in fa]); tb = np.array([f[0] for f in fb])
    pos = {}
    for j, v in enumerate(vb):
        pos.setdefault(int(v), []).append(j)
    deltas = []
    for i, v in enumerate(va):
        for j in pos.get(int(v), ()):
            if fa[i][2] == fb[j][2]:
                deltas.append(int(tb[j] - ta[i]))
    if not deltas:
        return dict(error="no common samples in the capture window")
    T = frame_period_ticks(fs, rate)
    k_all = np.rint(np.array(deltas) / T).astype(int)
    vals, cnt = np.unique(k_all, return_counts=True)
    k = int(vals[np.argmax(cnt)])
    sel = np.array(deltas)[k_all == k]
    return dict(frames=k, matches=int(len(sel)), delta_ticks=float(np.median(sel)),
                delta_ns=float(np.median(sel) / fs * 1e9),
                spread_ticks=int(sel.max() - sel.min()))


def residual_offset(a: dict, b: dict, fs: float, rate: int) -> dict | None:
    """Frame-start offset of b relative to a, folded into one frame period (no sample matching)."""
    ta = np.array([f[0] for f in a.get("frames", [])], dtype=np.float64)
    tb = np.array([f[0] for f in b.get("frames", [])], dtype=np.float64)
    if len(ta) < 2 or len(tb) < 2:
        return None
    T = frame_period_ticks(fs, rate)
    d = []
    for t in ta:
        j = np.searchsorted(tb, t)
        for jj in (j - 1, j):
            if 0 <= jj < len(tb):
                x = tb[jj] - t
                x -= T * np.rint(x / T)
                d.append(x)
    d = np.array(d)
    # Fold around the median so a residual near half a frame does not split.
    m = np.median(d)
    d = d - T * np.rint((d - m) / T)
    return dict(residual_ticks=float(np.median(d)), residual_ns=float(np.median(d) / fs * 1e9),
                spread_ticks=float(d.max() - d.min()), n=int(len(d)))
