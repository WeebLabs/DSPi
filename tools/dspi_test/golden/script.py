"""
Golden-capture request script: generator and loader.

The replayed artifact is the generated script_<platform>.json, which is pure data in legacy
(firmware 1.1.6) terms. This module only builds it; see README.md for the field meanings.
"""

from __future__ import annotations

import hashlib
import json
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LEGACY = json.loads((HERE / "legacy_opcodes.json").read_text())["opcodes"]

PLATFORMS = {
    "RP2350": dict(id=1, nch=17, nin=8, nout=9, nspdif=4, npin=5, nouns=81, siggen_types=15,
                   rta_chunks=2, reconfig=True),
    "RP2040": dict(id=0, nch=7, nin=2, nout=5, nspdif=2, npin=3, nouns=81, siggen_types=15,
                   rta_chunks=2, reconfig=False),
}
BANDS = list(range(10)) + [20, 21, 22, 23]
XOVER = [20, 21, 22, 23]
SETTLE_EQ = {"delay_ms": 8}
SETTLE_CS = {"poll": "cs"}
SETTLE_FLASH = {"delay_ms": 250, "poll": "ready"}
SETTLE_DEFER = {"delay_ms": 60, "poll": "ready"}
# 1 kHz sine at -20 dBFS on outputs 0-1, staged only. The boot-time config (all zero) is
# rejected by SIGGEN_SET_CONFIG, so this is the closest settable baseline.
SIGGEN_CANON = struct.pack("<BBHHBBfIHHffff", 1, 0, 0x0003, 0, 0, 0, -20.0, 0, 0, 0, 1000.0, 0, 0, 0)


def f32(x):
    return struct.pack("<f", x)


def u8(x):
    return struct.pack("<B", x & 0xFF)


def u16(x):
    return struct.pack("<H", x & 0xFFFF)


def u32(x):
    return struct.pack("<I", x & 0xFFFFFFFF)


def _req(op, d, wvalue=0, payload=b"", length=0, **extra):
    if op not in LEGACY:
        raise KeyError(op)
    e = {"op": op, "code": LEGACY[op], "dir": d, "wvalue": wvalue,
         "payload": payload.hex(), "len": length}
    e.update(extra)
    return e


def R(op, wvalue=0, length=4):
    return _req(op, "R", wvalue, b"", length)


def W(op, payload, wvalue=0, **extra):
    return _req(op, "W", wvalue, payload, 0, **extra)


def WR(op, wvalue=0, length=1, **extra):
    return _req(op, "WR", wvalue, b"", length, **extra)


# ---------------------------------------------------------------- read list

def make_reads(p):
    nch, nin, nout = p["nch"], p["nin"], p["nout"]
    r = []
    a = r.append
    # Identity
    a(R("GET_SERIAL", 0, 16)); a(R("GET_PLATFORM", 0, 7)); a(R("GET_BUILD_INFO", 0, 64))
    # Status selectors, plus unknown ones (release builds answer zeros)
    for wv in list(range(34)) + [34, 47, 63]:
        a(R("GET_STATUS", wv, nch * 2 + 7 if wv == 9 else 4))
    a(R("GET_BUFFER_STATS", 0, 44)); a(R("GET_USB_ERROR_STATS", 0, 24))
    a(R("GET_CLOCK_DIAG", 0, 248))
    # EQ, every channel x band x param, then out-of-range addresses
    for ch in range(nch):
        for b in BANDS:
            for prm in range(6):
                a(R("GET_EQ_PARAM", (ch << 8) | (b << 3) | prm, 4))
    for ch, b, prm in [(0, 10, 0), (0, 19, 0), (0, 24, 0), (0, 31, 1), (nch, 0, 0), (0xFF, 0, 0),
                       (0, 0, 6), (0, 0, 7), (nch - 1, 23, 7)]:
        a(R("GET_EQ_PARAM", (ch << 8) | (b << 3) | prm, 4))
    for ch in range(nch):
        for b in BANDS:
            a(R("GET_BAND_BYPASS", (ch << 8) | b, 1))
    for ch, b in [(0, 10), (0, 24), (nch, 0), (0, 0xFF)]:
        a(R("GET_BAND_BYPASS", (ch << 8) | b, 1))
    # Gain, volume, bypass
    a(R("GET_PREAMP", 0, 4))
    for ch in list(range(nin)) + [nin, 0xFF]:
        a(R("GET_PREAMP_CH", ch, 4))
    for op, n in [("GET_MASTER_VOLUME", 4), ("GET_MASTER_VOLUME_MODE", 1), ("GET_SAVED_MASTER_VOLUME", 4),
                  ("GET_USER_VOLUME", 4), ("GET_USER_MUTE", 1), ("GET_BYPASS", 1)]:
        a(R(op, 0, n))
    for s in list(range(7)) + [7, 0x100]:
        a(R("GET_INPUT_SOURCE_NAME", s, 32))
    a(R("GET_DEVICE_NAME", 0, 32)); a(R("GET_DEVICE_NAME", 1, 32))
    for ch in list(range(nch)) + [nch]:
        a(R("GET_DELAY", ch, 4))
    # Loudness, crossfeed, leveller
    for op, n in [("GET_LOUDNESS", 1), ("GET_LOUDNESS_REF", 4), ("GET_LOUDNESS_INTENSITY", 4),
                  ("GET_LOUDNESS_MASK", 2), ("GET_CROSSFEED", 1), ("GET_CROSSFEED_PRESET", 1),
                  ("GET_CROSSFEED_FREQ", 4), ("GET_CROSSFEED_FEED", 4), ("GET_CROSSFEED_ITD", 1),
                  ("GET_CROSSFEED_OUTPUTS", 1), ("GET_LEVELLER_ENABLE", 1), ("GET_LEVELLER_AMOUNT", 4),
                  ("GET_LEVELLER_SPEED", 1), ("GET_LEVELLER_MAX_GAIN", 4), ("GET_LEVELLER_LOOKAHEAD", 1),
                  ("GET_LEVELLER_GATE", 4), ("GET_LEVELLER_MASKS", 2)]:
        a(R(op, 0, n))
    # Matrix and per-output controls
    for i in range(nin):
        for o in range(nout):
            a(R("GET_MATRIX_ROUTE", (i << 8) | o, 8))
    a(R("GET_MATRIX_ROUTE", (nin << 8), 8)); a(R("GET_MATRIX_ROUTE", nout, 8))
    for op, n in [("GET_OUTPUT_ENABLE", 1), ("GET_OUTPUT_GAIN", 4), ("GET_OUTPUT_MUTE", 1),
                  ("GET_OUTPUT_DELAY", 4), ("GET_CORE1_CONFLICT", 1)]:
        for o in list(range(nout)) + [nout]:
            a(R(op, o, n))
    a(R("GET_CORE1_MODE", 0, 1))
    # Limiter: per-output params, meters, status, bad addresses
    for o in range(nout):
        for k in range(4):
            a(R("LIMITER", (o << 8) | k, 4))
    a(R("LIMITER", 0x80, nout * 2)); a(R("LIMITER", 0x81, 4))
    for wv in [(nout << 8), 0xFF00, 0x0004, 0x0082]:
        a(R("LIMITER", wv, 4))
    # Inputs
    for op, n in [("GET_INPUT_SOURCE", 1), ("GET_SPDIF_RX_STATUS", 16), ("GET_SPDIF_RX_CH_STATUS", 24),
                  ("GET_INPUT_RATE", 8), ("GET_SPDIF_INPUT_CONFIG", 6), ("GET_I2S_INPUT_CHANNELS", 1),
                  ("GET_I2S_CLOCK_MODE", 1), ("GET_I2S_SLAVE_STATUS", 16), ("GET_I2S_CLOCK_PIN_MODE", 1),
                  ("GET_ADAT_INPUT_ENABLE", 1), ("GET_ADAT_INPUT_PIN", 1), ("GET_ADAT_INPUT_CLOCK_MODE", 1),
                  ("GET_ADAT_INPUT_STATUS", 20), ("GET_LG_SOUND_SYNC_ENABLE", 1),
                  ("GET_LG_SOUND_SYNC_STATUS", 16)]:
        a(R(op, 0, n))
    for i in range(5):
        a(R("GET_SPDIF_RX_PIN", i, 1))
        a(R("GET_I2S_RX_PIN", i, 1))
    # Outputs, pins, clocks
    for s in list(range(p["nspdif"])) + [p["nspdif"]]:
        a(R("GET_OUTPUT_TYPE", s, 1))
    for i in list(range(p["npin"])) + [p["npin"]]:
        a(R("GET_OUTPUT_PIN", i, 1))
    for role in range(3):
        a(R("GET_I2S_BCK_PIN", role, 1))
    for op, n in [("GET_MCK_ENABLE", 1), ("GET_MCK_PIN", 1), ("GET_MCK_MULTIPLIER", 1),
                  ("GET_ADAT_ENABLE", 1), ("GET_ADAT_PIN", 1), ("GET_ADAT_STATUS", 8),
                  ("GET_DAC_HW_MUTE_CONFIG", 16), ("GET_UART_CONFIG", 8), ("GET_I2C_CONFIG", 8),
                  ("GET_CTRL_IFACE_STATUS", 8)]:
        a(R(op, 0, n))
    a(R("OUTPUT_LATENCY", 0, 12)); a(R("OUTPUT_LATENCY", 1, 56)); a(R("OUTPUT_LATENCY", 2, 12))
    # Presets and persistence
    for s in list(range(10)) + [10]:
        a(R("PRESET_GET_NAME", s, 32))
    for op, n in [("PRESET_GET_DIR", 7), ("PRESET_GET_STARTUP", 3), ("GET_OUTPUT_CONFIG_MODE", 1),
                  ("PRESET_GET_ACTIVE", 1)]:
        a(R(op, 0, n))
    for ch in list(range(nch)) + [nch]:
        a(R("GET_CHANNEL_NAME", ch, 32))
    # Signal generator
    a(R("SIGGEN_GET_CONFIG", 0, 36)); a(R("SIGGEN_GET_STATUS", 0, 16)); a(R("SIGGEN_GET_CAPS", 0xFFFF, 8))
    for t in range(p["siggen_types"] + 1):
        a(R("SIGGEN_GET_CAPS", t, 62))
    # Subharmonic, psychoacoustic bass, tube, upmixer
    for op, n in [("GET_SUBHARM", 1), ("GET_SUBHARM_LOW", 4), ("GET_SUBHARM_HIGH", 4),
                  ("GET_SUBHARM_BOOST", 4), ("GET_SUBHARM_MASK", 2), ("GET_SUBHARM_HEADROOM", 4),
                  ("GET_SUBHARM_TOP", 4), ("GET_SUBHARM_SELECT", 1), ("GET_SUBHARM_METER", nout * 2),
                  ("GET_SUBHARM_SOLO", 1), ("GET_SUBHARM_LINK", 1), ("GET_SUBHARM_DEPTH", 4),
                  ("GET_SUBHARM_HOLD", 4), ("GET_SUBHARM_CEILING", 4), ("GET_PSYBASS", 1),
                  ("GET_PSYBASS_CUTOFF", 4), ("GET_PSYBASS_HARMONICS", 4), ("GET_PSYBASS_DRIVE", 4),
                  ("GET_PSYBASS_CHARACTER", 4), ("GET_PSYBASS_ORIGINAL", 4), ("GET_PSYBASS_MASK", 2)]:
        a(R(op, 0, n))
    for i in list(range(15)) + [0x0100]:
        a(R("GET_TUBE_PARAM", i, 4))
    a(R("UPMIX_GET_CONFIG", 0, 44)); a(R("UPMIX_GET_STATUS", 0, 16))
    for i in range(15):
        a(R("UPMIX_GET_PARAM", i, 4))
    # Spectrum analyser (band and bin reads auto-start it unless MANUAL is set)
    a(R("RTA_GET_CONFIG", 0, 12)); a(R("RTA_GET_CAPS", 0, 16))
    for c in range(1, p["rta_chunks"] + 2):
        a(R("RTA_GET_CAPS", c, 64))
    a(R("RTA_GET_CAPS", 0x100, 16))
    a(R("RTA_GET_STATUS", 0, 24))
    for ch in list(range(nch)) + [0x100]:
        a(R("RTA_GET_BANDS", ch, 82))
    for off in range(0, 576, 64):
        a(R("RTA_GET_BINS", off, 64))
    a(R("RTA_GET_BANDS_ALL", 0, 82 * nch))
    # Control Surfaces
    for s in [0, 1, 14, 15, 16]:
        a(R("GET_CS_AUX_STATE", s, 1))
        a(R("GET_CS_AUX_LEVEL", s, 2))
    a(R("GET_CS_AUX_STATE", 0xFFFF, 48))
    for g in range(9):
        a(R("GET_CS_GROUP", g, 40))
    for m in range(9):
        a(R("GET_CS_MACRO", m, 132))
    a(R("GET_CS_EXT_STATUS", 0, 24)); a(R("GET_CS_DISPLAY_CFG", 0, 16)); a(R("GET_CS_DISPLAY_STATUS", 0, 8))
    for pg in range(17):
        a(R("GET_CS_DISPLAY_PAGE", pg, 4))
    for s in range(17):
        a(R("GET_CS_BINDING", s, 24))
        a(R("GET_CS_NAME", s, 32))
        a(R("GET_CS_IR_CMD", s, 16))
    a(R("GET_CS_NAME", 0x0100, 32))
    a(R("GET_CS_CAPS", 0xFFFF, 52))
    for n in range(p["nouns"] + 1):
        a(R("GET_CS_CAPS", n, 12))
    a(R("GET_CS_STATUS", 0, 41)); a(R("CS_IR_LEARN", 2, 8))
    # Bulk state. The chunk read is one whole-struct transfer: a later-offset chunk depends on a
    # session that any other host's vendor request tears down, so it cannot be replayed reliably.
    a(R("GET_ALL_PARAMS_CHUNK", 0, 6136))
    a(R("GET_ALL_PARAMS", 0, 6136))
    return r


# ---------------------------------------------------------------- write script

def _eq_packet(ch, b, typ, byp, freq, q, gain, qp=None):
    pkt = struct.pack("<BBBBfff", ch, b, typ, byp, freq, q, gain)
    return pkt + (u16(qp) if qp is not None else b"")


def make_default_writes(p):
    nch, nin, nout = p["nch"], p["nin"], p["nout"]
    w = []
    a = w.append
    a(W("SET_PREAMP", f32(-3.5)))
    for ch in range(nin):
        a(W("SET_PREAMP_CH", f32(-(1.0 + 0.25 * ch)), ch))
    a(W("SET_MASTER_VOLUME", f32(-23.5))); a(W("SET_USER_VOLUME", f32(-7.5)))
    a(W("SET_USER_MUTE", u8(1))); a(W("SET_BYPASS", u8(1)))
    for ch in range(nch):
        a(W("SET_DELAY", f32(0.5 + 0.125 * ch), ch))
    for o in range(nout):
        a(W("SET_OUTPUT_DELAY", f32(2.0 + 0.25 * o), o))
    a(W("SET_OUTPUT_ENABLE", u8(0), 1)); a(W("SET_OUTPUT_ENABLE", u8(1), 2))
    a(W("SET_OUTPUT_ENABLE", u8(1), 3))
    a(W("SET_OUTPUT_ENABLE", u8(1), nout - 1))   # PDM: refused by the Core 1 interlock
    for o in range(nout):
        a(W("SET_OUTPUT_GAIN", f32(-(1.5 + 0.5 * o)), o))
    a(W("SET_OUTPUT_MUTE", u8(1), 1)); a(W("SET_OUTPUT_MUTE", u8(1), 3))
    for i in range(nin):
        for o in range(nout):
            en = 1 if (i + 2 * o) % 3 else 0
            pkt = struct.pack("<BBBBf", i, o, en, (i + o) % 2, -(0.5 * i + 0.25 * o + 0.125))
            a(W("SET_MATRIX_ROUTE", pkt))
    a(W("LIMITER", f32(250.0), 0xFF02))
    for o in range(nout):
        a(W("LIMITER", f32(float(o % 2)), (o << 8) | 0))
        a(W("LIMITER", f32(-(2.0 + 0.5 * o)), (o << 8) | 1))
        a(W("LIMITER", f32(50.0 + 25.0 * o), (o << 8) | 2))
    a(W("LIMITER", f32(2.0), ((nout - 3) << 8) | 3)); a(W("LIMITER", f32(2.0), ((nout - 2) << 8) | 3))
    a(W("SET_LOUDNESS", u8(1))); a(W("SET_LOUDNESS_REF", f32(72.5)))
    a(W("SET_LOUDNESS_INTENSITY", f32(135.0))); a(W("SET_LOUDNESS_MASK", u16(0x01A5)))
    a(W("SET_CROSSFEED", u8(1))); a(W("SET_CROSSFEED_PRESET", u8(3)))
    a(W("SET_CROSSFEED_FREQ", f32(950.0))); a(W("SET_CROSSFEED_FEED", f32(7.5)))
    a(W("SET_CROSSFEED_ITD", u8(0))); a(W("SET_CROSSFEED_OUTPUTS", u8(0x05)))
    a(W("SET_LEVELLER_ENABLE", u8(1))); a(W("SET_LEVELLER_AMOUNT", f32(65.0)))
    a(W("SET_LEVELLER_SPEED", u8(2))); a(W("SET_LEVELLER_MAX_GAIN", f32(18.5)))
    a(W("SET_LEVELLER_LOOKAHEAD", u8(0))); a(W("SET_LEVELLER_GATE", f32(-55.0)))
    a(W("SET_LEVELLER_MASKS", bytes([0x0F, 0x33])))
    a(W("SET_LG_SOUND_SYNC_ENABLE", u8(0)))
    a(W("SET_INPUT_SOURCE", u8(0), settle=SETTLE_DEFER))   # same as active: ACKed no-op
    # EQ: the device holds one pending packet, so every write waits for the main loop
    types = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    for ch in range(nch):
        for b in range(10):
            typ = types[(ch * 3 + b) % 11]
            freq = 40.0 + 190.0 * b + 5.0 * ch
            q = 0.5 + 0.125 * b + 0.0625 * (ch % 4)
            gain = -6.0 + 1.25 * b + 0.25 * (ch % 5)
            byp = 1 if (ch + b) % 7 == 3 else 0
            if typ == 11:
                pkt = _eq_packet(ch, b, typ, byp, freq, q, 30.0 + ch, qp=256 + 8 * ch)
            else:
                pkt = _eq_packet(ch, b, typ, byp, freq, q, gain)
            a(W("SET_EQ_PARAM", pkt, settle=SETTLE_EQ))
        if ch >= nin:
            for j, b in enumerate(XOVER):
                typ = 32 + ((ch * 5 + j * 7) % 32)
                byp = 1 if (ch + j) % 5 == 0 else 0
                pkt = _eq_packet(ch, b, typ, byp, 60.0 + 400.0 * j + 10.0 * ch, 0.75, 0.0)
                a(W("SET_EQ_PARAM", pkt, settle=SETTLE_EQ))
    for ch in range(nch):
        a(W("SET_BAND_BYPASS", u8(1), (ch << 8) | (ch % 10), settle=SETTLE_EQ))
        if ch >= nin:
            a(W("SET_BAND_BYPASS", u8(1), (ch << 8) | 21, settle=SETTLE_EQ))
    for ch in range(nch):
        a(W("SET_CHANNEL_NAME", ("Gold%02d" % ch).encode(), ch))
    # Subharmonic synthesizer
    a(W("SET_SUBHARM", u8(1))); a(W("SET_SUBHARM_LOW", f32(-6.5))); a(W("SET_SUBHARM_HIGH", f32(3.5)))
    a(W("SET_SUBHARM_BOOST", f32(2.5))); a(W("SET_SUBHARM_MASK", u16(0x0103)))
    a(W("SET_SUBHARM_TOP", f32(-12.0))); a(W("SET_SUBHARM_SELECT", u8(1)))
    a(W("SET_SUBHARM_SOLO", u8(1))); a(W("SET_SUBHARM_LINK", u8(0)))
    a(W("SET_SUBHARM_DEPTH", f32(75.0))); a(W("SET_SUBHARM_HOLD", f32(220.0)))
    a(W("SET_SUBHARM_CEILING", f32(-6.0)))
    # Psychoacoustic bass
    a(W("SET_PSYBASS", u8(1))); a(W("SET_PSYBASS_CUTOFF", f32(120.0)))
    a(W("SET_PSYBASS_HARMONICS", f32(-3.5))); a(W("SET_PSYBASS_DRIVE", f32(9.0)))
    a(W("SET_PSYBASS_CHARACTER", f32(30.0))); a(W("SET_PSYBASS_ORIGINAL", f32(-6.0)))
    a(W("SET_PSYBASS_MASK", u16(0x0081)))
    # Tube: type 5 loads indices 4-7, then writing 4-7 drops the type back to 0 (custom)
    for idx, val in [(0, 1.0), (1, 66.0), (2, 5.0), (3, -6.0), (4, 25.0), (5, -2.0), (6, 60.0),
                     (7, 35.0), (8, 2.0), (9, 0.0), (10, 5.0), (11, 120.0), (12, 80.0), (13, -2.5)]:
        a(W("SET_TUBE_PARAM", f32(val), idx))
    # Upmixer (RP2040 ACKs and ignores both)
    up = struct.pack("<BBBb10f", 1, 0, 1, 4, 70.0, 60.0, 40.0, 15.0, 300.0, 150.0, 12.0, 120.0,
                     8000.0, 45.0)
    a(W("UPMIX_SET_CONFIG", up))
    for pid, val in [(3, 66.0), (13, 3.25), (1, 2.0), (9, 9.5)]:
        a(W("UPMIX_SET_PARAM", f32(val), pid))
    # Spectrum analyser: MANUAL flag so state-B reads do not auto-start it
    a(W("RTA_SET_CONFIG", struct.pack("<BBHBBHBBH", 3, 0, 0x0006, 9, 0, 750, 25, 1, 0),
        settle={"delay_ms": 50}))
    a(WR("RTA_CONTROL", 1)); a(WR("RTA_CONTROL", 2)); a(WR("RTA_CONTROL", 0))
    # Signal generator: staged only, never started (START would put a sweep on the outputs)
    sg = struct.pack("<BBHHBBfIHHffff", 1, 4, 0x0011, 0x0010, 1, 0, -30.0, 2000, 3, 100,
                     100.0, 5000.0, 0.0, 0.0)
    a(W("SIGGEN_SET_CONFIG", sg)); a(WR("SIGGEN_CONTROL", 0)); a(WR("SIGGEN_CONTROL", 2))
    a(WR("CS_MACRO_FIRE", 7)); a(WR("CS_MACRO_FIRE", 0xFFFF)); a(WR("CS_IR_LEARN", 0))
    # Counters and latches
    a(WR("CLEAR_CLIPS", 0, 4)); a(WR("RESET_BUFFER_STATS", 1)); a(WR("RESET_USB_ERROR_STATS", 0))
    return w


def make_flash_writes(p):
    w = []
    a = w.append
    # Control Surfaces previews, one deferred SET at a time. Restored by rewriting the old records
    # and CS_SAVE, because CS_REVERT in 1.1.6 leaves stored non-button bindings down.
    btn = struct.pack("<BBBBBBBBBBhhhhHHBB", 1, 3, 4, 0, 28, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    a(W("SET_CS_BINDING", btn, 15, settle=SETTLE_CS))
    a(W("SET_CS_NAME", b"GoldenSlot14", 14, settle=SETTLE_CS))
    a(W("SET_CS_NAME", b"GoldBtn", 15, settle=SETTLE_CS))
    ir = struct.pack("<BBBBBBhhBBI", 3, 4, 0, 0, 0, 1, 0, 0, 0, 0, 0x00FF12ED)
    a(W("SET_CS_IR_CMD", ir, 3, settle=SETTLE_CS))
    grp = struct.pack("<BBBBI32s", 2, 0, 0, 0, 0x05, b"GoldGrp")
    a(W("SET_CS_GROUP", grp, 2, settle=SETTLE_CS))
    st0 = struct.pack("<BBBBBBhhH", 3, 5, 0, 0, 0, 0, 1, 0, 0)
    st1 = struct.pack("<BBBBBBhhH", 9, 4, 0, 0, 0, 0, 0, 0, 10)
    a(W("SET_CS_MACRO_STEP", st0, (0 << 8) | 5, settle=SETTLE_CS))
    a(W("SET_CS_MACRO_STEP", st1, (1 << 8) | 5, settle=SETTLE_CS))
    a(W("SET_CS_MACRO", struct.pack("<32sBBBB", b"GoldMacro", 2, 0, 0, 0), 5, settle=SETTLE_CS))
    disp = struct.pack("<BBHHBBHH", 0, 1, 40, 25, 200, 0x14, 90, 0)
    a(W("SET_CS_DISPLAY_CFG", disp, settle=SETTLE_CS))
    a(W("SET_CS_DISPLAY_PAGE", bytes([17, 1, 0, 0x09]), 9, settle=SETTLE_CS))
    a(W("SET_DEVICE_NAME", b"GoldenDSPi", settle=SETTLE_FLASH))
    for s in (0, 3, 6):
        a(W("SET_INPUT_SOURCE_NAME", ("GoldSrc%d" % s).encode(), s, settle=SETTLE_FLASH))
    a(W("PRESET_SET_NAME", b"GoldScratch", 9, settle=SETTLE_FLASH))
    a(W("PRESET_SET_STARTUP", bytes([1, 3]), settle=SETTLE_FLASH))
    a(W("SET_OUTPUT_CONFIG_MODE", u8(0), settle=SETTLE_FLASH))
    a(W("SET_MASTER_VOLUME_MODE", u8(1), settle=SETTLE_FLASH))
    a(WR("SAVE_MASTER_VOLUME", 0, 1, settle=SETTLE_FLASH))
    return w


def make_reconfig_writes(p):
    """Input-side and storage-only IO settings; legal only while USB is the source and no slot is I2S."""
    if not p["reconfig"]:
        return []
    w = []
    a = w.append
    a(W("SET_INPUT_RATE", u32(96000), settle=SETTLE_DEFER))
    a(W("SET_I2S_CLOCK_MODE", u8(1), settle=SETTLE_DEFER))
    a(WR("SET_ADAT_INPUT_CLOCK_MODE", 1, settle=SETTLE_DEFER))
    a(WR("SET_ADAT_INPUT_PIN", 11)); a(WR("SET_ADAT_INPUT_ENABLE", 1))
    a(WR("SET_SPDIF_INPUT_ENABLE", (2 << 8) | 1))
    a(WR("SET_SPDIF_RX_PIN", (0 << 8) | 20)); a(WR("SET_SPDIF_RX_PIN", (3 << 8) | 19))
    a(WR("SET_I2S_RX_PIN", (1 << 8) | 18)); a(WR("SET_I2S_INPUT_CHANNELS", 4))
    a(WR("SET_I2S_BCK_PIN", (1 << 8) | 16)); a(WR("SET_I2S_BCK_PIN", (0 << 8) | 26))
    a(WR("SET_I2S_CLOCK_PIN_MODE", 1))
    a(WR("SET_MCK_MULTIPLIER", 1)); a(WR("SET_MCK_PIN", 15))
    a(WR("SET_ADAT_PIN", 4, settle=SETTLE_DEFER))
    return w


def make_baseline(p):
    """Requests that put transient state into a fixed shape after the baseline bulk SET."""
    return [
        _req("SET_ALL_PARAMS", "W", 0, b"", 0, payload_ref="baseline_bulk", settle={"delay_ms": 400, "poll": "ready"}),
        WR("SIGGEN_CONTROL", 2),
        W("SIGGEN_SET_CONFIG", SIGGEN_CANON),
        WR("RTA_CONTROL", 0),
        W("RTA_SET_CONFIG", struct.pack("<BBHBBHBBH", 3, 1, 0x0003, 10 if p["id"] == 1 else 9, 0, 300, 12, 0, 0),
          settle={"delay_ms": 50}),
        W("SET_SUBHARM_SOLO", u8(0)),
        WR("CLEAR_CLIPS", 0, 4),
        WR("RESET_BUFFER_STATS", 1),
    ]


def build(platform):
    p = PLATFORMS[platform]
    doc = {
        "format": 1,
        "platform": platform,
        "platform_id": p["id"],
        "note": "Legacy 1.1.6 requests. dir R = IN read, W = OUT write, WR = write-as-read (IN).",
        "baseline": make_baseline(p),
        "reads": make_reads(p),
        "writes": {
            "default": make_default_writes(p),
            "flash": make_flash_writes(p),
            "reconfig": make_reconfig_writes(p),
        },
    }
    return doc


def script_path(platform):
    return HERE / f"script_{platform.lower()}.json"


def load(platform):
    path = script_path(platform)
    raw = path.read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def read_key(e):
    """Stable identity of a read; the mask and the capture file are keyed by it."""
    return f"{e['op']}@0x{e['wvalue']:04X}"


def _dump(doc):
    """One request per line, so a regenerated script diffs readably."""
    def block(entries, indent):
        pad = " " * indent
        return "[\n" + ",\n".join(pad + json.dumps(e) for e in entries) + "\n" + pad[:-1] + "]"

    head = {k: v for k, v in doc.items() if k not in ("baseline", "reads", "writes")}
    out = ["{"] + [f" {json.dumps(k)}: {json.dumps(v)}," for k, v in head.items()]
    out.append(f' "baseline": {block(doc["baseline"], 2)},')
    out.append(f' "reads": {block(doc["reads"], 2)},')
    tiers = [f'  {json.dumps(t)}: {block(v, 3)}' for t, v in doc["writes"].items()]
    out.append(' "writes": {\n' + ",\n".join(tiers) + "\n }")
    return "\n".join(out) + "\n}\n"


def main():
    for plat in PLATFORMS:
        doc = build(plat)
        keys = [read_key(e) for e in doc["reads"]]
        if len(keys) != len(set(keys)):
            sys.exit(f"{plat}: duplicate read keys")
        path = script_path(plat)
        path.write_text(_dump(doc))
        n = {t: len(v) for t, v in doc["writes"].items()}
        print(f"{path.name}: {len(doc['reads'])} reads, writes {n}")


if __name__ == "__main__":
    main()
