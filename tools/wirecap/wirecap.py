#!/usr/bin/env python3
"""Wire-level output test rig for DSPi (needs a DSPI_WIRECAP firmware build).

  wirecap.py matrix -o results.json        run the whole matrix, write JSON and a summary
  wirecap.py matrix --only master/mixed    run the cases whose name contains a substring
  wirecap.py compare a.json b.json         PASS/FAIL per case between two result files
  wirecap.py rates                         find the overflow-free sampler dividers

Everything it changes is runtime state; it restores what it touched at the end.
Rig description and limits: Documentation/Features/wirecap_test_rig.md.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
import traceback

from rig import Rig, RATES, BCK_PIN, LRCK_PIN, Capture
import decode

SLOT_PINS_DEFAULT = (6, 7, 8, 9)
I2S_RX_PIN = 11            # GPIO 1 belongs to the UART control port on the bench unit
COARSE_DIV = 8             # 38.4 MS/s at 307.2 MHz: about 30 frames at 48 kHz in 96 KB
FINE_DIV = 2               # 153.6 MS/s: the fastest stall-free rate with all 32 pins
SLOW_DIV = 4               # 76.8 MS/s on one pin: 10.2 ms, at least two Z preambles at 44.1 kHz
TYPES = {"all_spdif": (0, 0, 0, 0), "mixed": (0, 1, 0, 1), "all_i2s": (1, 1, 1, 1)}
# 2 fine ticks: each line's edge is quantized to one tick, so a difference moves by up to two.
TOL_TICKS_SAME_CLOCK = 2.0
# Slave mode, S/PDIF to I2S only: constant within one lock, but each relock lands it anew
# (baseline spread 1.6 us over 36 relocks). Derivation: wirecap_test_rig.md, "Tolerances".
CROSS_TOL_NS = 2000.0
SIGGEN_LEVEL_DB = -40.0


def log(*a):
    print(*a, flush=True)


class Matrix:
    def __init__(self, rig: Rig, slot_pins=SLOT_PINS_DEFAULT, cross_tol_ns: float = 0.0):
        self.r = rig
        self.d = rig.d
        self.pins = slot_pins
        self.cross_tol_ns = cross_tol_ns

    # ---- device control ----

    def siggen_start(self):
        # White noise, the same stream on outputs 0-7, RAW so no per-output EQ alters it.
        cfg = struct.pack("<BBHHBBfIHH4f", 1, 2, 0xFF, 0, 0x01, 0, SIGGEN_LEVEL_DB, 0, 0, 0, 0, 0, 0, 0)
        self.d.pset("MEAS.SIGGEN_CONFIG", cfg)
        self.d.pset("MEAS.SIGGEN_START")

    def siggen_running(self) -> bool:
        return self.d.pget("MEAS.SIGGEN_STATUS")[1] in (1, 2, 3)

    def types(self):
        return tuple(self.d.pget("OUTPUT.TYPE", i)[0] for i in range(4))

    def set_types(self, types):
        cur = self.types()
        for i, t in enumerate(types):
            if cur[i] != t:
                self.d.pset("OUTPUT.TYPE", bytes([t]), i)
                time.sleep(0.3)

    def mode(self) -> str:
        return "slave" if self.d.pget("CLOCK.I2S_CLOCK_MODE")[0] == 1 else "master"

    def to_master(self, rate):
        if self.mode() == "slave" or self.d.pget("INPUT.SOURCE")[0] != 0:
            self.d.pset("CLOCK.I2S_CLOCK_MODE", b"\x00")
            self.d.pset("INPUT.SOURCE", b"\x00")
            time.sleep(0.5)
        if self.r.clkgen_status()["on"]:
            self.r.clkgen(False)
        if self.r.rate() != rate:
            self.r.request_rate(rate)

    def to_slave(self, rate):
        self.r.clkgen(True, rate)
        if self.d.pget("INPUT.I2S_RX_PIN", 0)[0] != I2S_RX_PIN:
            self.d.pset("INPUT.I2S_RX_PIN", bytes([I2S_RX_PIN]), 0)
        if self.d.pget("INPUT.I2S_RX_ENABLE")[0] != 1:
            self.d.pset("INPUT.I2S_RX_ENABLE", b"\x01")
        if self.mode() != "slave":
            self.d.pset("CLOCK.I2S_CLOCK_MODE", b"\x01")
        if self.d.pget("INPUT.SOURCE")[0] != 2:
            self.d.pset("INPUT.SOURCE", b"\x02")

    def configure(self, mode, types, rate):
        if mode == "master":
            self.to_master(rate)
        else:
            self.to_slave(rate)
        self.set_types(types)
        ok = self.r.wait_settled(rate, slave=(mode == "slave"))
        if not self.siggen_running():
            self.siggen_start()
        time.sleep(0.6)
        return ok

    # ---- measurement ----

    def _capture(self, base, count, div, tries=3) -> Capture:
        for _ in range(tries):
            c = self.r.capture(base, count, div)
            if not c.stalled:
                return c
        return c

    def lines(self, cap: Capture, types, rate) -> dict:
        out = {}
        for slot, t in enumerate(types):
            pin = self.pins[slot]
            if t == 0:
                out[slot] = decode.spdif_decode(cap.line(pin), cap.rate_hz, rate)
            else:
                out[slot] = decode.i2s_decode(cap.line(BCK_PIN), cap.line(LRCK_PIN), cap.line(pin), cap.rate_hz)
        return out

    def measure(self, mode, types, rate) -> dict:
        res = dict(mode=mode, types=list(types), rate=rate)
        h0 = self.r.health()
        coarse = self._capture(0, 32, COARSE_DIV)
        fine = self._capture(0, 32, FINE_DIV)
        res["sampler"] = dict(clk_hz=coarse.clk_hz, coarse_div=coarse.div, fine_div=fine.div,
                              coarse_rate_hz=coarse.rate_hz, fine_rate_hz=fine.rate_hz,
                              coarse_stalled=coarse.stalled, fine_stalled=fine.stalled,
                              coarse_frames_window=len(coarse.samples) / coarse.rate_hz * rate)
        lc = self.lines(coarse, types, rate)
        lf = self.lines(fine, types, rate)
        wire = {}
        for slot, t in enumerate(types):
            info = {k: v for k, v in lc[slot].items() if k != "frames"}
            info["frames_decoded"] = len(lc[slot]["frames"])
            fr = lc[slot]["frames"]
            info["left_equals_right"] = all(f[1] == f[2] for f in fr) if fr else None
            if t == 0:
                slow = self._capture(self.pins[slot], 1, SLOW_DIV)
                s = decode.spdif_decode(slow.line(self.pins[slot]), slow.rate_hz, rate)
                info["slow"] = {k: v for k, v in s.items() if k != "frames"}
                info["slow"]["stalled"] = slow.stalled
                info["slow"]["frames_decoded"] = len(s["frames"])
            info["pass"], info["fail_reasons"] = wire_verdict(t, info, coarse.rate_hz)
            wire[str(slot)] = info
        res["wire"] = wire

        sig = {}
        for a in range(4):
            for b in range(a + 1, 4):
                m = decode.match_offset(lc[a], lc[b], coarse.rate_hz, rate)
                f = decode.residual_offset(lf[a], lf[b], fine.rate_hz, rate)
                e = dict(match=m, fine=f)
                if m and "frames" in m and f:
                    # Coarse and fine residuals must agree to a coarse tick, or the fold was wrong.
                    coarse_res = m["delta_ticks"] / coarse.rate_hz - m["frames"] / rate
                    e["frames"] = m["frames"]
                    e["residual_ns"] = round(f["residual_ns"], 2)
                    e["coarse_fine_agree"] = abs(coarse_res * 1e9 - f["residual_ns"]) < 1e9 / coarse.rate_hz * 1.5
                    e["same_clock"] = mode == "master" or types[a] == types[b]
                sig[f"{a}-{b}"] = e
        res["pairs"] = sig
        res["signature"] = {k: dict(frames=v.get("frames"), residual_ns=v.get("residual_ns"),
                                    same_clock=v.get("same_clock"))
                            for k, v in sig.items()}
        res["tol_ns"] = dict(same_clock=TOL_TICKS_SAME_CLOCK / fine.rate_hz * 1e9,
                             cross_clock=self.cross_tol_ns)
        h1 = self.r.health()
        res["health_delta"] = {k: h1[k] - h0[k] for k in h1}
        res["health_after"] = h1
        if mode == "slave":
            res["slave_status"] = self.r.slave_status()
            res["clkgen"] = self.r.clkgen_status()
        res["wire_pass"] = all(w["pass"] for w in wire.values())
        res["signature_complete"] = all(v["frames"] is not None for v in res["signature"].values())
        return res


def wire_verdict(t, info, fs) -> tuple[bool, list]:
    bad = []
    if info.get("errors"):
        bad += info["errors"]
    if t == 0:
        for k in ("bad_runs", "framing_breaks", "unknown_preambles", "bmc_violations", "parity_errors",
                  "xy_order_errors", "v_set"):
            if info.get(k):
                bad.append(f"{k}={info[k]}")
        s = info.get("slow", {})
        for k in ("bad_runs", "framing_breaks", "unknown_preambles", "bmc_violations", "parity_errors",
                  "xy_order_errors", "frame_gaps"):
            if s.get(k):
                bad.append(f"slow.{k}={s[k]}")
        if s.get("z_spacing") != [192]:
            bad.append(f"z_spacing={s.get('z_spacing')}")
        if not s.get("cs_rate_ok"):
            bad.append(f"cs_rate_code={s.get('cs_rate_code')}")
        if s.get("stalled"):
            bad.append("slow capture stalled")
    else:
        if info.get("bck_per_frame") != [64]:
            bad.append(f"bck_per_frame={info.get('bck_per_frame')}")
        if info.get("bck_per_left_half") != [32]:
            bad.append(f"bck_per_left_half={info.get('bck_per_left_half')}")
        if info.get("lrck_to_bck_fall_max_ticks", 99) > 1:
            bad.append(f"lrck_to_bck_fall={info.get('lrck_to_bck_fall_max_ticks')}")
        if info.get("data_to_bck_fall_max_ticks", 99) > 2:
            bad.append(f"data_to_bck_fall={info.get('data_to_bck_fall_max_ticks')}")
    if info.get("frames_decoded", 0) < 10:
        bad.append("too few frames")
    if info.get("left_equals_right") is False:
        bad.append("left != right")
    return not bad, bad


def sig_equal(a: dict, b: dict, tol: dict) -> tuple[bool, list]:
    diffs = []
    for pair, sa in a.items():
        sb = b.get(pair)
        if sb is None:
            diffs.append(f"{pair}: missing")
            continue
        if sa["frames"] is None or sb["frames"] is None:
            diffs.append(f"{pair}: no offset ({sa['frames']} vs {sb['frames']})")
            continue
        if sa["frames"] != sb["frames"]:
            diffs.append(f"{pair}: sample offset {sa['frames']} -> {sb['frames']}")
        t = tol["same_clock"] if sa.get("same_clock", True) else max(tol["same_clock"], tol["cross_clock"])
        if abs(sa["residual_ns"] - sb["residual_ns"]) > t:
            diffs.append(f"{pair}: time {sa['residual_ns']:.1f} -> {sb['residual_ns']:.1f} ns (tol {t:.1f})")
    return not diffs, diffs


# ---- the matrix ----

def build_cases(only=None):
    cases = []
    for mode in ("master", "slave"):
        for tname in TYPES:
            for rate in RATES:
                cases.append(dict(name=f"{mode}/{tname}/{rate}", kind="static", mode=mode, types=tname, rate=rate))
    for ev in ("type_switch", "rate_change", "master_slave_master", "bulk_reapply"):
        cases.append(dict(name=f"event/{ev}", kind="event", event=ev, mode="master", types="mixed", rate=48000))
    for g, cells in ((1, 0), (2, 0), (3, 1), (4, 1), (3, 5)):
        cases.append(dict(name=f"glitch/kind{g}" + (f"_k{cells}" if cells else ""), kind="glitch",
                          glitch=g, cells=cells, mode="slave", types="mixed", rate=48000))
    if only:
        cases = [c for c in cases if any(o in c["name"] for o in only)]
    return cases


GLITCH_NAMES = {1: "runt BCK pulse", 2: "missing BCK pulse", 3: "LRCLK edge early", 4: "LRCLK edge late"}


def run_case(m: Matrix, case: dict) -> dict:
    types = TYPES[case["types"]]
    out = dict(case)
    if case["kind"] == "static":
        out["settled"] = m.configure(case["mode"], types, case["rate"])
        out["result"] = m.measure(case["mode"], types, case["rate"])
        out["pass"] = out["settled"] and out["result"]["wire_pass"] and out["result"]["signature_complete"]
        return out

    out["settled"] = m.configure(case["mode"], types, case["rate"])
    before = m.measure(case["mode"], types, case["rate"])
    out["before"] = before
    steps = []
    if case["kind"] == "event":
        ev = case["event"]
        if ev == "type_switch":
            m.set_types((0, 0, 0, 1)); steps.append("slot 1 I2S -> S/PDIF")
            m.r.wait_settled(case["rate"]); time.sleep(0.5)
            m.set_types(types); steps.append("slot 1 S/PDIF -> I2S")
        elif ev == "rate_change":
            m.r.request_rate(96000); steps.append(f"rate {case['rate']} -> 96000")
            steps.append(f"settled={m.r.wait_settled(96000)}")
            m.r.request_rate(case["rate"]); steps.append(f"rate 96000 -> {case['rate']}")
        elif ev == "master_slave_master":
            steps.append(f"to slave, settled={m.configure('slave', types, case['rate'])}")
            m.configure("master", types, case["rate"]); steps.append("back to master")
        elif ev == "bulk_reapply":
            blob = m.d.pget("STORE.ALL_PARAMS", length=6252)
            m.d.pset("STORE.ALL_PARAMS", blob); steps.append(f"STORE.ALL_PARAMS re-applied ({len(blob)} B)")
        settled = m.r.wait_settled(case["rate"], slave=(case["mode"] == "slave"))
        steps.append(f"settled={settled}")
        if not m.siggen_running():
            m.siggen_start(); steps.append("signal generator restarted (the event stopped it)")
        time.sleep(0.6)
    else:
        s0 = m.r.slave_status()
        g0 = m.r.clkgen_status()
        m.r.glitch(case["glitch"], case["cells"])
        steps.append(f"injected {GLITCH_NAMES[case['glitch']]}" + (f", {case['cells']} cells" if case["cells"] else ""))
        t0 = time.monotonic()
        seen, s = set(), s0
        while time.monotonic() - t0 < 6:
            s = m.r.slave_status()
            seen.add(s["state"])
            if s["slip_count"] > s0["slip_count"] and s["state"] == "LOCKED":
                break
            time.sleep(0.03)
        g1 = m.r.clkgen_status()
        out["glitch"] = dict(slips=s["slip_count"] - s0["slip_count"], relocked=s["state"] == "LOCKED",
                             relock_s=round(time.monotonic() - t0, 3), states_seen=sorted(seen),
                             late_repairs=g1["late"] - g0["late"])
        time.sleep(0.6)
        if not m.siggen_running():
            m.siggen_start()
            time.sleep(0.4)
    out["steps"] = steps
    after = m.measure(case["mode"], types, case["rate"])
    out["after"] = after
    eq, diffs = sig_equal(before["signature"], after["signature"], after["tol_ns"])
    out["signature_equal"] = eq
    out["signature_diffs"] = diffs
    ok = out["settled"] and before["wire_pass"] and after["wire_pass"] and eq
    if case["kind"] == "glitch":
        gl = out["glitch"]
        ok = ok and gl["slips"] >= 1 and gl["relocked"] and gl["late_repairs"] == 0
    out["pass"] = ok
    return out


def snapshot(d) -> dict:
    return dict(
        types=[d.pget("OUTPUT.TYPE", i)[0] for i in range(4)],
        bck0=d.pget("CLOCK.I2S_BCK_PIN", 0)[0],
        clock_mode=d.pget("CLOCK.I2S_CLOCK_MODE")[0],
        pin_mode=d.pget("CLOCK.I2S_CLOCK_PIN_MODE")[0],
        source=d.pget("INPUT.SOURCE")[0],
        i2s_rx_enable=d.pget("INPUT.I2S_RX_ENABLE")[0],
        i2s_rx_pin0=d.pget("INPUT.I2S_RX_PIN", 0)[0],
        out_enable=[d.pget("ROUTE.OUTPUT_ENABLE", o)[0] for o in range(9)],
        mck=d.pget("CLOCK.MCK_ENABLE")[0],
        rate=Rig(d).rate(),
    )


def prepare(m: Matrix, snap: dict):
    if snap["mck"]:
        raise SystemExit("MCK is enabled; the rig keeps MCK off. Disable it first.")
    if snap["pin_mode"] != 0:
        raise SystemExit("clock-pin mode is SPLIT; the rig needs UNIFIED so slave mode listens on GPIO 24/25.")
    d = m.d
    if snap["types"] != [0, 0, 0, 0] and snap["bck0"] != BCK_PIN:
        # An I2S slot would drive the clocks on a header pin while BCK moves; start from all S/PDIF.
        m.set_types((0, 0, 0, 0))
    if d.pget("CLOCK.I2S_BCK_PIN", 0)[0] != BCK_PIN:
        d.pset("CLOCK.I2S_BCK_PIN", bytes([BCK_PIN]), 0)
    for o in range(8):
        if not snap["out_enable"][o]:
            d.pset("ROUTE.OUTPUT_ENABLE", b"\x01", o)
    m.siggen_start()


def restore(m: Matrix, snap: dict):
    d, r = m.d, m.r
    steps = []

    def step(desc, fn):
        try:
            fn()
            steps.append(desc)
        except Exception as e:  # keep going: every later step still matters
            steps.append(f"{desc}: FAILED {e}")

    step("siggen stop", lambda: d.pset("MEAS.SIGGEN_STOP_NOW"))
    step("clock mode", lambda: d.pset("CLOCK.I2S_CLOCK_MODE", bytes([snap["clock_mode"]])))
    step("source", lambda: d.pset("INPUT.SOURCE", bytes([snap["source"]])))
    time.sleep(0.5)
    step("clkgen off", lambda: r.clkgen(False))
    step("all S/PDIF", lambda: m.set_types((0, 0, 0, 0)))
    step("I2S RX enable", lambda: d.pset("INPUT.I2S_RX_ENABLE", bytes([snap["i2s_rx_enable"]])))
    step("I2S RX pin", lambda: d.pset("INPUT.I2S_RX_PIN", bytes([snap["i2s_rx_pin0"]]), 0))
    step("BCK pin", lambda: d.pset("CLOCK.I2S_BCK_PIN", bytes([snap["bck0"]]), 0))
    step("types", lambda: m.set_types(tuple(snap["types"])))
    for o in range(9):
        step(f"output {o} enable", lambda o=o: d.pset("ROUTE.OUTPUT_ENABLE", bytes([snap["out_enable"][o]]), o))
    if snap["source"] == 0:
        step("rate", lambda: r.request_rate(snap["rate"]) if r.rate() != snap["rate"] else None)
    time.sleep(1.0)
    return steps


def summarize(doc: dict) -> str:
    lines = [f"wirecap matrix  firmware {doc['firmware']}  {doc['started']}",
             f"sampler: clk_sys {doc['sampler_note']}", ""]
    for c in doc["cases"]:
        verdict = "PASS" if c.get("pass") else "FAIL"
        lines.append(f"{verdict}  {c['name']}")
        if "error" in c:
            lines.append(f"      error: {c['error'].splitlines()[-1]}")
            continue
        res = c.get("result") or c.get("after")
        if res:
            for slot, w in res["wire"].items():
                if not w["pass"]:
                    lines.append(f"      slot {slot}: {', '.join(w['fail_reasons'])}")
            sig = res["signature"]
            def fmt(v):
                if v["frames"] is None:
                    return "none"
                return f"{v['frames']}f{v['residual_ns']:+.0f}ns"
            lines.append("      offsets " + "  ".join(f"{k}:{fmt(v)}" for k, v in sig.items()))
            hd = {k: v for k, v in res["health_delta"].items() if v}
            if hd:
                lines.append(f"      counters moved: {hd}")
        if c.get("signature_diffs"):
            lines.append(f"      signature changed: {'; '.join(c['signature_diffs'])}")
        if "glitch" in c:
            lines.append(f"      glitch: {c['glitch']}")
        if c.get("steps"):
            lines.append(f"      steps: {'; '.join(c['steps'])}")
    n = sum(1 for c in doc["cases"] if c.get("pass"))
    lines += ["", f"{n}/{len(doc['cases'])} cases pass"]
    return "\n".join(lines)


def cmd_matrix(args):
    r = Rig()
    fw = r.d.pget("SYS.BUILD_INFO").split(b"\0")[0].decode()
    m = Matrix(r, cross_tol_ns=args.cross_tol_ns)
    snap = snapshot(r.d)
    log(f"firmware {fw}; device state before: {snap}")
    doc = dict(firmware=fw, started=time.strftime("%Y-%m-%d %H:%M:%S"), device_before=snap,
               settings=dict(coarse_div=COARSE_DIV, fine_div=FINE_DIV, slow_div=SLOW_DIV,
                             siggen_level_db=SIGGEN_LEVEL_DB, i2s_rx_pin=I2S_RX_PIN,
                             slot_pins=list(m.pins), cross_tol_ns=args.cross_tol_ns), cases=[])
    try:
        prepare(m, snap)
        for case in build_cases(args.only):
            log(f"== {case['name']}")
            try:
                res = run_case(m, case)
            except Exception:
                res = dict(case, error=traceback.format_exc(), **{"pass": False})
                log(res["error"])
            doc["cases"].append(res)
            log("   " + ("PASS" if res.get("pass") else "FAIL"))
    finally:
        doc["restore_steps"] = restore(m, snap)
        doc["device_after"] = snapshot(r.d)
        log(f"restored: {doc['device_after']}")
    clk = doc["cases"][0].get("result", {}).get("sampler", {}).get("clk_hz") if doc["cases"] else None
    doc["sampler_note"] = (f"{clk} Hz, coarse {clk / COARSE_DIV / 1e6:.1f} MS/s, fine {clk / FINE_DIV / 1e6:.1f} MS/s, "
                           f"slow {clk / SLOW_DIV / 1e6:.1f} MS/s (1 pin)") if clk else "n/a"
    with open(args.output, "w") as f:
        json.dump(doc, f, indent=1, default=lambda o: int(o) if hasattr(o, "__int__") else str(o))
    text = summarize(doc)
    summary_path = args.summary or args.output.rsplit(".", 1)[0] + "_summary.txt"
    with open(summary_path, "w") as f:
        f.write(text + "\n")
    print(text)


def case_sig(c):
    if "result" in c:
        return [("result", c["result"])]
    return [(k, c[k]) for k in ("before", "after") if k in c]


def cmd_compare(args):
    a, b = (json.load(open(p)) for p in (args.a, args.b))
    bc = {c["name"]: c for c in b["cases"]}
    fails = 0
    print(f"A: {a['firmware']}\nB: {b['firmware']}\n")
    for ca in a["cases"]:
        cb = bc.get(ca["name"])
        why = []
        if cb is None:
            why.append("missing in B")
        else:
            if not cb.get("pass"):
                why.append("B's own checks fail" + (f" ({'; '.join(cb.get('signature_diffs', []))})"
                                                     if cb.get("signature_diffs") else ""))
            for (ka, ra), (kb, rb) in zip(case_sig(ca), case_sig(cb)):
                tol = dict(same_clock=max(ra["tol_ns"]["same_clock"], rb["tol_ns"]["same_clock"]),
                           cross_clock=max(ra["tol_ns"]["cross_clock"], rb["tol_ns"]["cross_clock"],
                                           args.cross_tol_ns))
                eq, diffs = sig_equal(ra["signature"], rb["signature"], tol)
                if not eq:
                    why.append(f"{kb}: " + "; ".join(diffs))
                for slot, wa in ra["wire"].items():
                    wb = rb["wire"].get(slot, {})
                    if wa.get("pass") and not wb.get("pass"):
                        why.append(f"{kb} slot {slot}: {', '.join(wb.get('fail_reasons', ['missing']))}")
                    if wa.get("slow", {}).get("cs_a") != wb.get("slow", {}).get("cs_a"):
                        why.append(f"{kb} slot {slot}: channel status {wa.get('slow', {}).get('cs_a')} -> "
                                   f"{wb.get('slow', {}).get('cs_a')}")
        if not ca.get("pass"):
            why.append("(A failed this case too)")
        ok = not [w for w in why if not w.startswith("(A failed")] or (not ca.get("pass") and not why[:-1])
        fails += 0 if ok else 1
        print(f"{'PASS' if ok else 'FAIL'}  {ca['name']}" + ("" if ok and not why else "\n      " + "\n      ".join(why)))
    print(f"\n{len(a['cases']) - fails}/{len(a['cases'])} cases PASS")
    return 1 if fails else 0


def cmd_rates(args):
    r = Rig()
    for count in (32, 4, 1):
        for div in (1, 2, 3, 4, 8):
            stalls = sum(r.capture(0 if count == 32 else 6, count, div).stalled for _ in range(args.repeat))
            print(f"pins {count:2d}  div {div}  {r.cap_status()['clk_hz'] / div / 1e6:7.1f} MS/s  "
                  f"stalled in {stalls}/{args.repeat} captures")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("matrix")
    p.add_argument("-o", "--output", default="wirecap_results.json")
    p.add_argument("--summary")
    p.add_argument("--only", nargs="*")
    p.add_argument("--cross-tol-ns", type=float, default=CROSS_TOL_NS,
                   help="extra tolerance for S/PDIF-to-I2S pairs in slave mode (see the rig doc)")
    p.set_defaults(fn=cmd_matrix)
    p = sub.add_parser("compare")
    p.add_argument("a")
    p.add_argument("b")
    p.add_argument("--cross-tol-ns", type=float, default=CROSS_TOL_NS)
    p.set_defaults(fn=cmd_compare)
    p = sub.add_parser("rates")
    p.add_argument("--repeat", type=int, default=5)
    p.set_defaults(fn=cmd_rates)
    args = ap.parse_args()
    sys.exit(args.fn(args) or 0)


if __name__ == "__main__":
    main()
