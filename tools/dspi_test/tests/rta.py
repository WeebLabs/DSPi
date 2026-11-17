"""
Spectrum analyser (MEAS.RTA_*), section 8 of Documentation/Features/spectrum_analyser_spec.md.
The analyser is transient and outside the bulk snapshot, so every test stops it and the generator in
a finally. The taps only run while the pipeline does, so level tests drive it with a generator signal.
"""

import struct
import time

from ..device import PagedStall, Stall
from ..framework import Skip, test
from ..helpers import expect_stall, has_command, pipeline_stats, sample_rate, wait_for

# --- rta.h wire constants ---------------------------------------------------

RTA_CFG_VERSION = 3
RTA_TAP_INPUT, RTA_TAP_OUTPUT = 0, 1
RTA_STATE_IDLE, RTA_STATE_CAPTURING, RTA_STATE_TRANSFORMING = 0, 1, 2
RTA_CH_NONE = 0xFF
RTA_BAND_FRAME = 82
RTA_BINS_PER_READ = 256      # MEAS.RTA_BINS reply ceiling
RTA_BIN_HEADER = 16

# --- siggen.h wire constants (the generator needs no host audio, and its pump
# drives the pipeline when nothing streams) ------------------------------------

SIGGEN_CFG_VERSION = 1
SIGGEN_SINE, SIGGEN_PINK = 0, 3
SIGGEN_FLAG_RAW = 0x01

# --- Tolerances -------------------------------------------------------------

SINE_HZ = 1000.0
SINE_DBFS = -20.0
SINE_TOL_DB = 1.0
SINE_REJECT_DBFS = -60.0     # bands two or more away from the tone
PINK_DBFS = -20.0
PINK_AVG_MS = 2000
PINK_SPREAD_DB = 2.0
PINK_MIN_BINS = 12           # below this the integer bin edges tilt the band
AGE_SLACK = 1.5              # x the measured rotation interval
# MEAS.RTA_BANDS reads are serialised over EP0, so a whole-set sweep costs a few ms
# per channel; without this the age bound fails on read latency, not staleness.
AGE_READ_SLACK_MS = 50.0
# Ceiling on the transform's summed slice time per frame, as a share of the frame period, by
# platform_id. Headroom over the measured ~5% (RP2350, 48 kHz) so 96 kHz still passes.
FRAME_SHARE = {1: 0.15, 0: 0.30}
BUSY_SETTLE_S = 8.0          # busy_us_per_s is a 1 s-window EMA at quarter weight
AUTO_OFF_WAIT_S = 6.0        # > RTA_IDLE_TIMEOUT_MS
LEVEL_FLOOR_DBFS = -200.0    # what a level byte of 0 means to the assertions

SUPPORTED_RATES = (44100, 48000, 96000)


# --- Wire helpers -----------------------------------------------------------

def _caps(dev):
    """MEAS.RTA_CAPS as a dict. Raises Skip on firmware without the analyser."""
    if not has_command(dev, "MEAS.RTA_CAPS", "GET"):
        raise Skip("spectrum analyser not present on this firmware")
    raw = dev.pget("MEAS.RTA_CAPS", length=16)
    (version, n_in, n_out, o_min, o_max, o_def, bass_bands, max_bands,
     level_zero, dyn_db, idle_ms, max_bin, bass_dyn_db) = struct.unpack("<10BHHH", raw)
    if version != RTA_CFG_VERSION:
        raise Skip(f"RTA caps version {version}, harness speaks {RTA_CFG_VERSION}")
    return {"input_channels": n_in, "output_channels": n_out,
            "order_min": o_min, "order_max": o_max, "order_default": o_def,
            "max_bands": max_bands, "bass_bands": bass_bands,
            "bass_dynamic_range_db": bass_dyn_db,
            "level_zero": level_zero, "dynamic_range_db": dyn_db,
            "idle_timeout_ms": idle_ms, "max_bin_frame": max_bin}


def _centres(dev, caps):
    """Band centre frequencies in Hz from MEAS.RTA_BAND_CENTRES, 32 bands per chunk."""
    out = []
    chunk = 0
    while len(out) < caps["max_bands"]:
        raw = dev.pget("MEAS.RTA_BAND_CENTRES", chunk, 64)
        if not raw:
            break
        out += list(struct.unpack(f"<{len(raw) // 2}H", raw[:len(raw) // 2 * 2]))
        chunk += 1
    return out[:caps["max_bands"]]


def _config(tap, mask, order, avg_ms=250, peak_db_s=20, flags=0):
    return struct.pack("<BBHBBHBBH", RTA_CFG_VERSION, tap, mask, order, 0,
                       avg_ms, peak_db_s, flags, 0)


def _get_config(dev):
    raw = dev.pget("MEAS.RTA_CONFIG", length=12)
    (version, tap, mask, order, _reserved, avg_ms, peak, flags, _r) = struct.unpack("<BBHBBHBBH", raw)
    return {"version": version, "tap": tap, "mask": mask, "order": order,
            "avg_ms": avg_ms, "peak_decay_db_s": peak, "flags": flags}


def _status(dev):
    raw = dev.pget("MEAS.RTA_STATUS", length=24)
    (version, state, tap, fast_ch, _res0, live_count, live_mask, frames_per_s,
     busy, last_us, idle_ms, fs, first_band, _r, bass_busy) = struct.unpack("<6B5HIBBH", raw)
    return {"version": version, "state": state, "tap": tap,
            "fast_channel": fast_ch,
            "live_count": live_count, "live_mask": live_mask,
            "frames_per_s": frames_per_s, "busy_us_per_s": busy,
            "last_frame_us": last_us, "idle_ms": idle_ms, "sample_rate_hz": fs,
            "first_band": first_band, "bass_busy_us_per_s": bass_busy}


def _bands(dev, ch, caps):
    """One channel's MEAS.RTA_BANDS frame, levels already decoded to dBFS."""
    raw = dev.pget("MEAS.RTA_BANDS", ch, RTA_BAND_FRAME)
    version, channel, seq, n_bands, age, _res = struct.unpack("<BBBBHH", raw[:8])
    nb = caps["max_bands"]
    avg = raw[8:8 + nb]
    peak = raw[8 + nb:8 + 2 * nb]
    return {"version": version, "channel": channel, "seq": seq,
            "n_bands": n_bands, "age_ms": age,
            "avg_db": [_level_db(v, caps) for v in avg[:n_bands]],
            "peak_db": [_level_db(v, caps) for v in peak[:n_bands]],
            "avg_raw": list(avg)}


def _level_db(v, caps):
    """Wire level byte to dBFS.  0 is the floor, not -121.5 dB of real signal."""
    return LEVEL_FLOOR_DBFS if v == 0 else (v - caps["level_zero"]) * 0.5


def _stop_rta(dev):
    try:
        dev.pset("MEAS.RTA_STOP")
    except (Stall, OSError):
        pass


def _stop_siggen(dev):
    try:
        dev.pset("MEAS.SIGGEN_STOP_NOW")
    except (Stall, OSError):
        pass


def _siggen(dev, sig_type, mask, level_db, p1=0.0):
    """Hold a continuous generator signal on `mask`, RAW so the output-side tap
    sees the generator itself and not whatever PEQ the previous test left."""
    try:
        dev.pset("MEAS.SIGGEN_CONFIG",
                 struct.pack("<BBHHBBfIHHffff", SIGGEN_CFG_VERSION, sig_type, mask,
                             0, SIGGEN_FLAG_RAW, 0, level_db, 0, 0, 0,
                             p1, 0.0, 0.0, 0.0))
        dev.pset("MEAS.SIGGEN_START")
    except PagedStall as e:
        raise Skip(f"onboard signal generator unavailable: {e}")
    dev.wait_ready()


# --- Setup / teardown of the levels the output tap sees ---------------------

def _open_outputs(dev, outs):
    """Enable and unmute `outs` at unity, with both volumes at 0 dB, so the
    output tap reads the generator's own level.  Returns the saved state."""
    saved = {"enable": {}, "mute": {}, "gain": {},
             "master": dev.pget("ROUTE.MASTER_VOLUME", length=4),
             "user": dev.pget("ROUTE.USER_VOLUME", length=4),
             "user_mute": dev.pget("ROUTE.USER_MUTE", length=1)}
    for o in outs:
        saved["enable"][o] = dev.pget("ROUTE.OUTPUT_ENABLE", o, 1)
        saved["mute"][o] = dev.pget("ROUTE.OUTPUT_MUTE", o, 1)
        saved["gain"][o] = dev.pget("ROUTE.OUTPUT_GAIN", o, 4)
    try:
        for o in outs:
            try:
                dev.pset_u8("ROUTE.OUTPUT_ENABLE", 1, o)
            except PagedStall as s:
                # VC_ERR_STATE: the core 1 mode cannot run this output too; it stays off and out of the live mask.
                if s.result_name != "VC_ERR_STATE":
                    raise
            dev.pset_u8("ROUTE.OUTPUT_MUTE", 0, o)
            dev.pset_f32("ROUTE.OUTPUT_GAIN", 0.0, o)
        dev.pset_f32("ROUTE.MASTER_VOLUME", 0.0)
        dev.pset_f32("ROUTE.USER_VOLUME", 0.0)
        dev.pset_u8("ROUTE.USER_MUTE", 0)
    except BaseException:
        _restore_outputs(dev, saved)
        raise
    dev.wait_ready()
    return saved


def _restore_outputs(dev, saved):
    # Disables first, so re-enabling an output never collides with one this test enabled.
    order = sorted(saved["enable"], key=lambda o: saved["enable"][o] != b"\x00")
    for o in order:
        for cmd, key in (("ROUTE.OUTPUT_ENABLE", "enable"), ("ROUTE.OUTPUT_MUTE", "mute"),
                         ("ROUTE.OUTPUT_GAIN", "gain")):
            try:
                dev.pset(cmd, saved[key][o], o)
            except (Stall, OSError):
                pass
    for cmd, key in (("ROUTE.MASTER_VOLUME", "master"), ("ROUTE.USER_VOLUME", "user"),
                     ("ROUTE.USER_MUTE", "user_mute")):
        try:
            dev.pset(cmd, saved[key])
        except (Stall, OSError):
            pass
    dev.wait_ready()


# --- Band geometry ----------------------------------------------------------

def _rate(dev):
    fs = sample_rate(dev)
    if fs not in SUPPORTED_RATES:
        raise Skip(f"no RTA band table at {fs} Hz (supported: {SUPPORTED_RATES})")
    return fs


def _band_of(centres, hz):
    """Index of the third-octave band whose nominal centre is nearest `hz`."""
    return min(range(len(centres)), key=lambda i: abs(centres[i] - hz))


def _band_bins(centre_hz, fs, order):
    """Roughly how many fast-stream bins a band spans, for deciding which bands
    the transform actually resolves (see tools/rta_test/README.md)."""
    lo = centre_hz * 2.0 ** (-1.0 / 6.0)
    hi = centre_hz * 2.0 ** (1.0 / 6.0)
    return (hi - lo) / (fs / float(1 << order))


def _gradeable_bands(centres, n_bands, fs, order):
    """Bands a flatness test may fairly grade: fully below Nyquist and wide
    enough that the integer bin edges are not the dominant error."""
    out = []
    for b in range(min(n_bands, len(centres))):
        if centres[b] * 2.0 ** (1.0 / 6.0) >= fs / 2.0:
            continue                       # Nyquist truncates the top band
        if _band_bins(centres[b], fs, order) < PINK_MIN_BINS:
            continue                       # coarser than the band
        out.append(b)
    return out


# --- Polling ----------------------------------------------------------------

def _poll_bands(dev, ch, caps, seconds, period=0.25):
    """Read one channel's bands every `period` for `seconds`, returning the last
    frame.  Band reads double as the auto-off keepalive, so a dwell has to poll:
    a plain sleep longer than RTA_IDLE_TIMEOUT_MS stops the analyser."""
    deadline = time.monotonic() + seconds
    frame = _bands(dev, ch, caps)
    while time.monotonic() < deadline:
        time.sleep(period)
        frame = _bands(dev, ch, caps)
    return frame


def _await_frames(dev, ch, caps, timeout_s=4.0):
    """Poll until this channel has produced a fast-stream frame.  Returns the
    frame, or None if none arrived (no source and no generator driving blocks)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        f = _bands(dev, ch, caps)
        if f["age_ms"] != 0xFFFF:
            return f
        time.sleep(0.1)
    return None


def _apply_config(dev, cfg):
    """SET MEAS.RTA_CONFIG and wait until the main loop has applied it (the GET shows the applied one)."""
    dev.pset("MEAS.RTA_CONFIG", cfg)
    return wait_for(lambda: dev.pget("MEAS.RTA_CONFIG", length=12), cfg, timeout_s=1.0)


def _start(dev, cfg):
    """Apply a config, then start the engine, so START never arms the previous config."""
    _apply_config(dev, cfg)
    dev.pset("MEAS.RTA_START")
    time.sleep(0.1)


# --- Tests ------------------------------------------------------------------

@test("rta", mutating=True)
def rta_config_validation(dev, profile, chk):
    """MEAS.RTA_CONFIG refuses an empty mask and an out-of-range order with VC_ERR_VALUE; a valid one applies."""
    caps = _caps(dev)
    chk.in_range(caps["order_default"], caps["order_min"], caps["order_max"],
                 "caps default order inside the caps range")
    chk.eq(caps["output_channels"], profile.num_output_channels, "caps output channel count")
    chk.eq(caps["input_channels"], profile.num_input_channels, "caps input channel count")
    try:
        good = _config(RTA_TAP_OUTPUT, (1 << profile.num_output_channels) - 1,
                       caps["order_default"], avg_ms=250, peak_db_s=20)
        chk.no_stall(lambda: _apply_config(dev, good), "valid config accepted")
        applied = _get_config(dev)
        chk.eq(applied["tap"], RTA_TAP_OUTPUT, "tap applied")
        chk.eq(applied["order"], caps["order_default"], "order applied")
        chk.eq(applied["mask"], (1 << profile.num_output_channels) - 1, "mask applied")

        bad = [(_config(RTA_TAP_OUTPUT, 0, caps["order_default"]), "empty channel mask"),
               (_config(RTA_TAP_OUTPUT, 0x1, caps["order_max"] + 1),
                f"order {caps['order_max'] + 1} above the caps ceiling"),
               (_config(RTA_TAP_OUTPUT, 0x1, caps["order_min"] - 1),
                f"order {caps['order_min'] - 1} below the caps floor"),
               # Only bits above the tap's width: empty after masking.
               (_config(RTA_TAP_OUTPUT, 0xFFFF << profile.num_output_channels & 0xFFFF,
                        caps["order_default"]), "mask with no in-range bit")]
        for cfg, label in bad:
            expect_stall(chk, lambda: dev.pset("MEAS.RTA_CONFIG", cfg), "VC_ERR_VALUE", label)
        # The rejected configs must not have displaced the applied one.
        chk.eq(_get_config(dev)["order"], caps["order_default"],
               "applied config survives the rejected ones")
    finally:
        _stop_rta(dev)


@test("rta", mutating=True)
def rta_sine_lands_in_its_band(dev, profile, chk):
    """A -20 dBFS 1 kHz tone reads -20 dBFS in its band and below -60 dBFS two bands away."""
    caps = _caps(dev)
    fs = _rate(dev)
    centres = _centres(dev, caps)
    order = caps["order_default"]
    out = min(2, profile.num_output_channels - 1)
    saved = _open_outputs(dev, [out])
    try:
        _siggen(dev, SIGGEN_SINE, 1 << out, SINE_DBFS, p1=SINE_HZ)
        _start(dev, _config(RTA_TAP_OUTPUT, 1 << out, order, avg_ms=250))
        st = _status(dev)
        if not (st["live_mask"] & (1 << out)):
            raise Skip(f"output {out} is not live (live_mask 0x{st['live_mask']:04X})")
        if _await_frames(dev, out, caps) is None:
            raise Skip("no RTA frame completed; the pipeline is not running")
        frame = _poll_bands(dev, out, caps, 1.5)

        b1k = _band_of(centres, SINE_HZ)
        chk.ok(b1k < frame["n_bands"], f"1 kHz band index {b1k} inside {frame['n_bands']} bands")
        if b1k >= frame["n_bands"]:
            return
        level = frame["avg_db"][b1k]
        chk.approx(level, SINE_DBFS, SINE_TOL_DB,
                   f"{centres[b1k]} Hz band reads the tone ({level:.2f} dBFS)")

        far = [(b, frame["avg_db"][b]) for b in range(frame["n_bands"])
               if abs(b - b1k) >= 2]
        worst_b, worst = max(far, key=lambda bl: bl[1]) if far else (None, LEVEL_FLOOR_DBFS)
        chk.ok(worst < SINE_REJECT_DBFS,
               f"loudest band two or more away is {centres[worst_b]} Hz at "
               f"{worst:.1f} dBFS (want < {SINE_REJECT_DBFS:g})")
        chk.note(f"sine: {centres[b1k]} Hz band {level:.2f} dBFS, "
                 f"worst far band {worst:.1f} dBFS, order {order}, {fs} Hz")
    finally:
        _stop_rta(dev)
        _stop_siggen(dev)
        _restore_outputs(dev, saved)


@test("rta", mutating=True)
def rta_pink_noise_flat(dev, profile, chk):
    """Pink noise reads flat within 2 dB across the resolved bands, on every selected channel."""
    caps = _caps(dev)
    fs = _rate(dev)
    centres = _centres(dev, caps)
    order = caps["order_default"]
    outs = list(range(profile.num_output_channels))
    mask = (1 << profile.num_output_channels) - 1
    saved = _open_outputs(dev, outs)
    try:
        _siggen(dev, SIGGEN_PINK, mask, PINK_DBFS)
        _start(dev, _config(RTA_TAP_OUTPUT, mask, order, avg_ms=PINK_AVG_MS))
        st = _status(dev)
        live = [o for o in outs if st["live_mask"] & (1 << o)]
        if not live:
            raise Skip(f"no live outputs (live_mask 0x{st['live_mask']:04X})")
        if _await_frames(dev, live[0], caps) is None:
            raise Skip("no RTA frame completed; the pipeline is not running")

        # Averaging is a power-domain EMA with a PINK_AVG_MS time constant, so
        # dwell past two time constants before reading.
        dwell = 2.5 * PINK_AVG_MS / 1000.0
        deadline = time.monotonic() + dwell
        while time.monotonic() < deadline:
            _bands(dev, live[0], caps)
            time.sleep(0.25)

        graded = _gradeable_bands(centres, caps["max_bands"], fs, order)
        if not graded:
            raise Skip(f"no band at {fs} Hz order {order} spans "
                       f"{PINK_MIN_BINS} bins")
        for o in live:
            f = _bands(dev, o, caps)
            usable = [b for b in graded if b < f["n_bands"]]
            levels = [f["avg_db"][b] for b in usable]
            if not levels or min(levels) <= LEVEL_FLOOR_DBFS:
                chk.ok(False, f"output {o}: a graded band read the wire floor "
                              f"(bands {usable}, levels {[round(x, 1) for x in levels]})")
                continue
            spread = max(levels) - min(levels)
            chk.ok(spread <= PINK_SPREAD_DB,
                   f"output {o}: pink spread {spread:.2f} dB over "
                   f"{centres[usable[0]]}-{centres[usable[-1]]} Hz "
                   f"(want <= {PINK_SPREAD_DB:g})")
        chk.note(f"pink flatness graded on bands "
                 f"{centres[graded[0]]}-{centres[graded[-1]]} Hz "
                 f"({len(graded)} of {caps['max_bands']}) at {fs} Hz order {order}")
    finally:
        _stop_rta(dev)
        _stop_siggen(dev)
        _restore_outputs(dev, saved)


@test("rta", mutating=True)
def rta_rotation_refreshes_every_channel(dev, profile, chk):
    """With every live output selected, seq advances and age_ms stays inside 1.5x the rotation."""
    caps = _caps(dev)
    _rate(dev)
    order = caps["order_default"]
    outs = list(range(profile.num_output_channels))
    mask = (1 << profile.num_output_channels) - 1
    saved = _open_outputs(dev, outs)
    try:
        _siggen(dev, SIGGEN_PINK, mask, PINK_DBFS)
        _start(dev, _config(RTA_TAP_OUTPUT, mask, order, avg_ms=250))
        st = _status(dev)
        live = [o for o in outs if st["live_mask"] & (1 << o)]
        if not live:
            raise Skip(f"no live outputs (live_mask 0x{st['live_mask']:04X})")
        if _await_frames(dev, live[0], caps) is None:
            raise Skip("no RTA frame completed; the pipeline is not running")

        first = {o: _bands(dev, o, caps)["seq"] for o in live}
        # One full rotation is live_count frames; give it two, plus a floor so a
        # very fast rotation still gets a usable settling window.
        st = _status(dev)
        chk.eq(st["live_count"], len(live), "live_count matches the live mask")
        fps = st["frames_per_s"]
        rotation_ms = (1000.0 * len(live) / fps) if fps else 1000.0
        deadline = time.monotonic() + max(1.0, 2.5 * rotation_ms / 1000.0)
        while time.monotonic() < deadline:
            for o in live:
                _bands(dev, o, caps)
            time.sleep(0.1)

        limit_ms = AGE_SLACK * rotation_ms + AGE_READ_SLACK_MS
        for o in live:
            f = _bands(dev, o, caps)
            chk.ne(f["seq"], first[o], f"output {o} seq advanced (was {first[o]})")
            chk.ok(f["age_ms"] < limit_ms,
                   f"output {o} age {f['age_ms']} ms within {limit_ms:.0f} ms "
                   f"(rotation {rotation_ms:.0f} ms over {len(live)} channels)")
        chk.note(f"rotation: {len(live)} channels, {fps} frames/s, "
                 f"{rotation_ms:.0f} ms per channel")
    finally:
        _stop_rta(dev)
        _stop_siggen(dev)
        _restore_outputs(dev, saved)


@test("rta", mutating=True)
def rta_busy_within_budget(dev, profile, chk):
    """At order 10 no background step overruns, and each frame's summed transform time stays a small share of the frame."""
    caps = _caps(dev)
    _rate(dev)
    order = 10
    if order > caps["order_max"]:
        raise Skip(f"caps order_max is {caps['order_max']}, no order 10")
    share = FRAME_SHARE.get(profile.platform_id)
    if share is None:
        raise Skip(f"no transform budget for platform id {profile.platform_id}")
    outs = list(range(profile.num_output_channels))
    mask = (1 << profile.num_output_channels) - 1
    saved = _open_outputs(dev, outs)
    try:
        _siggen(dev, SIGGEN_PINK, mask, PINK_DBFS)
        _start(dev, _config(RTA_TAP_OUTPUT, mask, order, avg_ms=250))
        st = _status(dev)
        live = [o for o in outs if st["live_mask"] & (1 << o)]
        if not live:
            raise Skip(f"no live outputs (live_mask 0x{st['live_mask']:04X})")
        if _await_frames(dev, live[0], caps) is None:
            raise Skip("no RTA frame completed; the pipeline is not running")
        # busy_us_per_s is an EMA over 1 s windows at quarter weight, so it needs
        # several seconds before it means anything.
        worst = 0
        over0 = pipeline_stats(dev)["bg_over_budget_count"]
        deadline = time.monotonic() + BUSY_SETTLE_S
        while time.monotonic() < deadline:
            _bands(dev, live[0], caps)
            worst = max(worst, _status(dev)["last_frame_us"])
            time.sleep(0.25)
        st = _status(dev)
        fps = st["frames_per_s"]
        over = pipeline_stats(dev)["bg_over_budget_count"] - over0
        # last_frame_us sums the slices' run time, not wall time, so it is a CPU cost per frame.
        period_us = 1e6 * (1 << order) / st["sample_rate_hz"]
        budget = int(share * period_us)
        chk.ok(over == 0, f"{over} background steps over budget while the transform ran")
        chk.ok(worst < budget, f"worst last_frame_us {worst} under {budget} us, {share:.0%} of the "
                               f"{period_us:.0f} us frame ({profile.platform_name}, order {order})")
        # The EMA lags, so grade it against what the per-frame budget allows at the measured rate.
        chk.ok(st["busy_us_per_s"] < budget * max(fps, 1),
               f"busy_us_per_s {st['busy_us_per_s']} under {budget} us x {fps} frames/s")
        chk.note(f"busy: {st['busy_us_per_s']} us/s EMA, last frame {st['last_frame_us']} us "
                 f"(worst {worst}), {fps} frames/s of {st['sample_rate_hz'] / (1 << order):.1f} possible, "
                 f"{over} background steps over budget; bass taps: {st['bass_busy_us_per_s']} us/s summed "
                 "over both cores (65535 means saturated; excludes packet bookkeeping)")
    finally:
        _stop_rta(dev)
        _stop_siggen(dev)
        _restore_outputs(dev, saved)


@test("rta", mutating=True)
def rta_auto_off_and_restart(dev, profile, chk):
    """Six seconds without a data read returns the engine to idle; one band read restarts it."""
    caps = _caps(dev)
    _rate(dev)
    outs = list(range(profile.num_output_channels))
    mask = (1 << profile.num_output_channels) - 1
    saved = _open_outputs(dev, outs)
    try:
        _start(dev, _config(RTA_TAP_OUTPUT, mask, caps["order_default"], avg_ms=250))
        _bands(dev, 0, caps)          # a read, so the idle timer starts from here
        st = _status(dev)
        if st["state"] == RTA_STATE_IDLE:
            raise Skip(f"engine will not arm (live_mask 0x{st['live_mask']:04X}, "
                       f"{st['sample_rate_hz']} Hz)")

        # MEAS.RTA_STATUS is not a keepalive, so polling it here must not keep the analyser alive.
        deadline = time.monotonic() + AUTO_OFF_WAIT_S
        while time.monotonic() < deadline:
            time.sleep(0.5)
            _status(dev)
        st = _status(dev)
        chk.eq(st["state"], RTA_STATE_IDLE,
               f"idle after {AUTO_OFF_WAIT_S:g} s without a data read "
               f"(timeout {caps['idle_timeout_ms']} ms, idle_ms {st['idle_ms']})")
        chk.eq(st["fast_channel"], RTA_CH_NONE, "no channel captured while idle")

        _bands(dev, 0, caps)
        time.sleep(0.2)
        chk.ne(_status(dev)["state"], RTA_STATE_IDLE, "one band read restarts the engine")
    finally:
        _stop_rta(dev)
        _restore_outputs(dev, saved)


@test("rta", mutating=True)
def rta_bin_frame_seq_consistency(dev, profile, chk):
    """MEAS.RTA_BINS read in 256-byte pieces: seq head equals tail (re-read on a turnover); past the end is VC_ERR_ADDRESS."""
    caps = _caps(dev)
    fs = _rate(dev)
    order = caps["order_default"]
    out = min(2, profile.num_output_channels - 1)
    saved = _open_outputs(dev, [out])
    try:
        _siggen(dev, SIGGEN_SINE, 1 << out, SINE_DBFS, p1=SINE_HZ)
        _start(dev, _config(RTA_TAP_OUTPUT, 1 << out, order, avg_ms=250))
        if _await_frames(dev, out, caps) is None:
            raise Skip("no RTA frame completed; the pipeline is not running")

        frame, attempts = _read_bin_frame(dev, chk)
        if frame is None:
            return
        chk.eq(frame["version"], RTA_CFG_VERSION, "bin frame version")
        chk.eq(frame["channel"], out, "bin frame channel is the rotated channel")
        chk.eq(frame["fft_order"], order, "bin frame order matches the config")
        chk.eq(frame["sample_rate_hz"], fs, "bin frame sample rate")
        chk.eq(frame["n_bins"], (1 << order) // 2, "n_bins is N/2")
        chk.ok(frame["total"] <= caps["max_bin_frame"],
               f"frame {frame['total']} B inside the advertised "
               f"{caps['max_bin_frame']} B maximum")
        chk.eq(frame["bytes"][2], frame["bytes"][frame["total"] - 1],
               "seq head equals seq tail")
        expect_stall(chk, lambda: dev.pget("MEAS.RTA_BINS", frame["total"], 16), "VC_ERR_ADDRESS",
                     "offset past the end of the frame")
        chk.note(f"bin frame: {frame['total']} B, {frame['n_bins']} bins, "
                 f"seq {frame['bytes'][2]}, {attempts} read attempt(s)")
    finally:
        _stop_rta(dev)
        _stop_siggen(dev)
        _restore_outputs(dev, saved)


def _read_bin_frame(dev, chk, tries=12):
    """Read the whole bin frame in 256-byte pieces (offsets 0, 256, 512), retrying while head and tail
    disagree. The device takes no lock, so a turnover mid-read is expected; the retry gap varies so the
    reads cannot stay in phase with the publish cadence."""
    seen = []
    for attempt in range(1, tries + 1):
        buf = bytearray(dev.pget("MEAS.RTA_BINS", 0, RTA_BINS_PER_READ))
        if len(buf) < RTA_BIN_HEADER:
            time.sleep(0.1)
            continue
        (version, channel, seq, order, fs, n_bins, _r0, _r1,
         _r2) = struct.unpack_from("<BBBBIHHHH", buf)
        if version != RTA_CFG_VERSION or n_bins == 0:
            time.sleep(0.1)              # zero header: no transform published yet
            continue
        total = RTA_BIN_HEADER + n_bins + 1
        while len(buf) < total:
            piece = dev.pget("MEAS.RTA_BINS", len(buf), RTA_BINS_PER_READ)
            if not piece:
                break
            buf += piece
        if len(buf) == total and buf[2] == buf[total - 1]:
            return {"version": version, "channel": channel, "seq": seq,
                    "fft_order": order, "sample_rate_hz": fs, "n_bins": n_bins,
                    "total": total, "bytes": bytes(buf)}, attempt
        seen.append((buf[2], buf[-1]))
        time.sleep(0.003 + 0.006 * (attempt % 4))
    chk.ok(False, f"bin frame seq head never matched its tail in {tries} reads (head, tail): {seen}")
    return None, tries


@test("rta", mutating=True)
def rta_input_tap_usb_sine(dev, profile, chk):
    """Switching the tap to the input side puts a USB-played 1 kHz tone in the 1 kHz input band."""
    try:
        from .. import audio
    except ImportError as e:      # pragma: no cover - harness without the audio module
        raise Skip(f"audio helpers unavailable: {e}")
    import threading

    caps = _caps(dev)
    fs = _rate(dev)
    centres = _centres(dev, caps)
    order = caps["order_default"]
    try:
        out_dev, in_dev, _info = audio.find_devices()
    except audio.AudioUnavailable as e:
        raise Skip(f"host audio playback unavailable: {e}")

    in_mask = (1 << profile.num_input_channels) - 1
    errors = []
    tone = audio.make_tone(fs, SINE_HZ, dur_s=5.0, amp=10.0 ** (SINE_DBFS / 20.0))

    def _play():
        try:
            audio.play_record(tone, fs, out_dev, in_dev, in_channels=2, out_channels=2)
        except Exception as e:      # noqa: BLE001 - reported as a Skip below
            errors.append(e)

    player = threading.Thread(target=_play, daemon=True)
    player.start()
    try:
        _start(dev, _config(RTA_TAP_INPUT, in_mask, order, avg_ms=250))
        time.sleep(1.0)             # let playback reach steady state
        st = _status(dev)
        chk.eq(st["tap"], RTA_TAP_INPUT, "status reports the input tap")
        if errors:
            raise Skip(f"USB playback failed: {errors[0]}")
        if not st["live_mask"]:
            raise Skip(f"no live input channel (live_mask 0x{st['live_mask']:04X})")
        ch = (st["live_mask"] & -st["live_mask"]).bit_length() - 1
        if _await_frames(dev, ch, caps) is None:
            raise Skip("no RTA frame completed on the input tap")
        frame = _poll_bands(dev, ch, caps, 1.5)

        b1k = _band_of(centres, SINE_HZ)
        loudest = max(range(frame["n_bands"]), key=lambda b: frame["avg_db"][b])
        chk.eq(loudest, b1k,
               f"loudest input band is {centres[b1k]} Hz "
               f"(got {centres[loudest]} Hz at {frame['avg_db'][loudest]:.1f} dBFS)")
        chk.ok(frame["avg_db"][b1k] > SINE_REJECT_DBFS,
               f"1 kHz input band carries the tone "
               f"({frame['avg_db'][b1k]:.1f} dBFS)")
        chk.note(f"input tap ch{ch}: {centres[b1k]} Hz band "
                 f"{frame['avg_db'][b1k]:.1f} dBFS at {fs} Hz")
    finally:
        _stop_rta(dev)
        player.join(timeout=10.0)
