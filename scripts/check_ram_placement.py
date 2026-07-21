#!/usr/bin/env python3
"""Verify hot audio/USB code is RAM-resident after the copy_to_ram -> XIP switch.

Runs four checks against one or more linked ELFs and exits non-zero on any FAIL:
  A  hot symbols must live in RAM (nm --print-size)
  B  no hot function may branch/call into flash (objdump BFS from RAM roots)
  B2 core-1 function bodies must hold no flash-range literals (.word scan)
  C  RAM/flash size accounting vs scripts/ram_baseline.json and .data budgets

Usage: python3 scripts/check_ram_placement.py <elf> [<elf> ...]
Platform (rp2040 / rp2350) and the loopback variant are auto-detected from the
ELF path. Requires arm-none-eabi-nm / -objdump / -size on PATH (stdlib only).
"""

import bisect
import json
import os
import re
import subprocess
import sys

# ---------------------------------------------------------------------------
# Address map and per-platform budgets
# ---------------------------------------------------------------------------
RAM_LO, RAM_HI = 0x20000000, 0x20090000     # valid RAM window (brackets both parts)
FLASH_LO, FLASH_HI = 0x10000000, 0x20000000  # XIP flash window
ROM_HI = 0x00008000                          # bootrom below here is always fine

RAM_LEN = {"rp2040": 262144, "rp2350": 524288}
# rp2350 raised 64K -> 72K for the ADAT input receiver (2026-07-13): the
# RAM-pinned decode path (adat_input_poll + rate machine + shared servo)
# adds ~3 KB of deliberately hot code.
DATA_BUDGET = {"rp2040": 61440, "rp2350": 73728}

# Flash callees that are cold-path-only and safe for a hot caller to reference.
WHITELIST = {
    "panic", "panic_unsupported", "__assert_func", "hard_assert",
    "assert_failed", "isr_hardfault", "__wrap_printf", "weak_raw_printf",
    "stdio_flush", "hard_assertion_failure",
}

# ---------------------------------------------------------------------------
# Hot-symbol table. Each entry is (name, static_ok). static_ok => a "not found"
# result is a WARN (compiler may have inlined it) rather than a FAIL.
# One-line additions keep this table the single source of truth.
# ---------------------------------------------------------------------------
COMMON = [
    ("process_input_block", False),
    ("uac1_driver_xfer_cb", True),
    ("uac1_driver_sof", True),
    ("process_audio_packet", True),
    ("fb_ctrl_sof_update", False),
    ("fb_ctrl_get_10_14", False),
    ("usb_audio_drain_ring", False),
    ("usb_audio_flush_ring", False),
    ("spdif_input_poll", False),
    ("spdif_input_update_clock_servo", False),
    ("i2s_input_poll", False),
    ("pdm_core1_entry", False),
    ("pdm_processing_loop", True),
    ("pdm_push_sample", False),
    ("pdm_get_dma_fill_pct", False),
    ("pdm_get_ring_fill_pct", False),
    ("eq_worker_loop", True),
    ("spdif_rx_dma_irq_handler", True),
    ("_check_block", True),
    ("_dma_done_and_restart", True),
    ("spdif_rx_callback_func", False),
    # Output DMA IRQ handlers (primary streaming interrupts).
    ("audio_spdif_dma_irq_handler", True),
    ("audio_i2s_dma_irq_handler", True),
    ("take_audio_buffer", False),
    ("give_audio_buffer", False),
    ("get_free_audio_buffer", True),
    ("get_full_audio_buffer", True),
    ("queue_full_audio_buffer", True),
    ("queue_free_audio_buffer", True),
    ("usbd_edpt_xfer", False),
    ("dcd_int_handler", True),
    ("dcd_event_handler", False),
    ("tud_task_ext", False),
    # TinyUSB OSAL event queue is tu_fifo based (pico_util queue is not linked).
    ("tu_fifo_read", False),
    ("tu_fifo_write", False),
    ("tu_edpt_claim", False),
    ("tu_edpt_release", False),
    # Alarm pool + timer: cancelled and re-armed per block by the SPDIF RX ISR
    # (alarm_pool_add_alarm_in_us is a header inline over _add_alarm_at).
    ("alarm_pool_cancel_alarm", False),
    ("alarm_pool_add_alarm_at", False),
    ("time_us_64", False),
    ("hardware_alarm_set_target", True),
    # Steady-state servo and SPDIF-input poll path.
    ("clock_get_hz", False),
    ("audio_i2s_mck_set_divider", False),
    # Live-tail variants; the plain read_fifo/get_fifo_count are internal-only
    # since the input path went word-granular and may be inlined/gc'd away.
    ("spdif_rx_read_fifo_live", False),
    ("spdif_rx_get_fifo_count_live", False),
    # Notify ring: drained from the USB ISR while parameters change.
    ("notify_has_pending_for", False),
    ("notify_peek_next_for", False),
    ("notify_commit_pop_for", False),
    ("multicore_lockout_handler", True),
    ("dspi_flash_range_erase", False),
    ("dspi_flash_range_program", False),
    ("__wrap_powf", False),
    ("__wrap_log10f", False),
    ("dsp_process_channel_block", False),
    ("xover_process_channel_block", False),
    ("crossfeed_process_pair_block", False),
    ("crossfeed_process_pairs", False),
    ("leveller_process_block", False),
    ("update_buffer_watermarks", True),
    ("get_slot_consumer_fill", False),
    # Signal generator: siggen_render runs in the block pipeline; the synth
    # helpers below are file-static and may be inlined (static_ok).
    ("siggen_render", False),
    ("osc_sin", True),
    ("blep", True),
    ("synth_sine", True),
    ("synth_square", True),
    ("synth_white", True),
    ("synth_pink", True),
    ("synth_sweep", True),
    ("synth_click", True),
    ("synth_polarity", True),
    ("synth_burst", True),
    ("synth_tone_pair", True),
    ("synth_multitone", True),
    ("synth_isp", True),
    ("synth_channel_id", True),
    ("mark_stopped", True),
    ("segment_finished", True),
    ("synth_reset_cycle", True),
    ("current_cycle_samples", True),
    ("window_gain_at", True),
    ("fade_gain_of", True),
    ("next_boundary", True),
    ("plan_block", True),
    ("synth_segment", True),
]

RP2350_EXTRA = [
    ("__wrap_expf", True),
    ("__wrap_logf", True),
    # RP2350-only: QMI clkdiv helper and the separate C dsp kernel entry
    # (folded into dsp_process_channel_block on RP2040).
    ("dspi_set_clkdiv", False),
    ("dsp_process_channel", False),
    # Compiler/newlib mem ops; kept in RAM by the linker script exclusions.
    ("memset", False),
    ("memcpy", False),
    ("memmove", False),
]

# RP2040 float / integer-divider shims. Their linker names vary by build; the
# names below were confirmed as DEFINED symbols in build-rp2040/DSPi/DSPi.elf
# (all resolved to RAM: fadd/fdiv/fmul/f2iz/i2f at 0x20004xxx, idiv==idivmod at
# 0x20003d78, uidiv==uidivmod at 0x20003dc8). The plain __aeabi_* variants are
# not defined here (only the __wrap_ interposers are), so those are used.
RP2040_EXTRA = [
    ("dsp_process_band_cascade_block", True),
    # ROM-backed mem_ops shims (PICO_MEM_IN_RAM=1); newlib copies are unlinked.
    ("__wrap_memset", False),
    ("__wrap_memcpy", False),
    ("__wrap___aeabi_fadd", False),
    ("__wrap___aeabi_fmul", False),
    ("__wrap___aeabi_fdiv", False),
    ("__wrap___aeabi_f2iz", False),
    ("__wrap___aeabi_i2f", False),
    ("__wrap___aeabi_idiv", False),
    ("__wrap___aeabi_idivmod", False),
    ("__wrap___aeabi_uidiv", False),
    ("__wrap___aeabi_uidivmod", False),
]

LOOPBACK_EXTRA = [
    ("loopback_push_slot0", False),
    ("fill_audio_packet", True),
    ("lb_arm_in", True),
    ("lb_driver_xfer_cb", True),
]

# Core-1 roots for the literal-pool scan (Check B2).
CORE1_ROOTS = ["pdm_core1_entry", "pdm_processing_loop", "eq_worker_loop"]

BRANCH_MNEMONICS = {"bl", "blx", "b", "b.w", "b.n"}

# Check B2: flash literals in core-1 code that are provably init-time only.
# Core 1 dereferences these once at launch, never during a flash write window.
B2_LITERAL_WHITELIST = {
    "multicore_lockout_victim_init",
    "__VECTOR_TABLE",
    "__vectors",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def in_ram(a):
    return RAM_LO <= a < RAM_HI

def in_flash(a):
    return FLASH_LO <= a < FLASH_HI

def detect_platform(path):
    p = path.lower()
    if "rp2350" in p:
        return "rp2350"
    return "rp2040"  # default; build-rp2040 and build-rp2040-loopback both land here

def is_loopback(path):
    return "loopback" in path.lower()

def hot_list(platform, loopback):
    entries = list(COMMON)
    entries += RP2350_EXTRA if platform == "rp2350" else RP2040_EXTRA
    if loopback:
        entries += LOOPBACK_EXTRA
    return entries

def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    except FileNotFoundError:
        sys.exit("error: %s not found; arm-none-eabi binutils must be on PATH" % cmd[0])

# ---------------------------------------------------------------------------
# Symbol table (nm --print-size --defined-only)
# ---------------------------------------------------------------------------

def load_symbols(elf):
    """Return list of dicts {name, addr, size} for all defined symbols."""
    out = run(["arm-none-eabi-nm", "--print-size", "--defined-only", elf])
    syms = []
    for line in out.splitlines():
        t = line.split()
        if len(t) >= 4 and re.fullmatch(r"[0-9a-fA-F]+", t[1] or ""):
            addr, size, name = int(t[0], 16), int(t[1], 16), " ".join(t[3:])
        elif len(t) == 3:
            addr, size, name = int(t[0], 16), 0, t[2]
        else:
            continue
        syms.append({"name": name, "addr": addr, "size": size})
    return syms

def match_symbols(syms, name):
    """Symbols whose name equals `name` or begins with `name.` (compiler suffixes)."""
    return [s for s in syms if s["name"] == name or s["name"].startswith(name + ".")]

def nearest_symbol(sorted_addrs, addr):
    """Name of the defined symbol at or just below addr (thumb bit stripped)."""
    a = addr & ~1
    i = bisect.bisect_right(sorted_addrs, (a, "￿")) - 1
    return sorted_addrs[i][1] if i >= 0 else ""

# ---------------------------------------------------------------------------
# Disassembly parsing (objdump -d --no-show-raw-insn)
# ---------------------------------------------------------------------------

LABEL_RE = re.compile(r"^([0-9a-fA-F]+) <(.+)>:$")
INSN_RE = re.compile(r"^([0-9a-fA-F]+):\t(\S+)\t?(.*)$")
TARGET_RE = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>")

def parse_disasm(elf):
    """Parse objdump into per-function bodies.

    Returns (funcs, starts) where funcs[start] = {name, end, insns, words} and
    starts is the sorted list of function start addresses. insns is a list of
    (addr, mnemonic, operand); words is a list of (addr, value) from .word lines.
    """
    out = run(["arm-none-eabi-objdump", "-d", "--no-show-raw-insn", elf])
    funcs = {}
    cur = None
    for line in out.splitlines():
        m = LABEL_RE.match(line)
        if m:
            start = int(m.group(1), 16)
            cur = {"name": m.group(2), "start": start, "end": start,
                   "insns": [], "words": []}
            funcs[start] = cur
            continue
        m = INSN_RE.match(line)
        if not m or cur is None:
            continue
        addr, mnem, operand = int(m.group(1), 16), m.group(2), m.group(3).strip()
        cur["end"] = addr + 2  # provisional; refined from sorted starts below
        if mnem == ".word":
            wm = re.match(r"0x([0-9a-fA-F]+)", operand)
            if wm:
                cur["words"].append((addr, int(wm.group(1), 16)))
        elif mnem in BRANCH_MNEMONICS:
            cur["insns"].append((addr, mnem, operand))
    starts = sorted(funcs)
    # Refine each function's end to the next function start (exclusive range).
    for i, s in enumerate(starts):
        funcs[s]["end"] = starts[i + 1] if i + 1 < len(starts) else s + funcs[s]["end"]
    return funcs, starts

def enclosing_func(starts, funcs, addr):
    """Function-start address whose [start, end) range contains addr, or None."""
    i = bisect.bisect_right(starts, addr) - 1
    if i < 0:
        return None
    s = starts[i]
    if funcs[s]["start"] <= addr < funcs[s]["end"]:
        return s
    return None

def parse_target(operand):
    """Return (kind, addr, name). kind is 'addr' | 'indirect' | 'none'."""
    m = TARGET_RE.match(operand)
    if m:
        base = m.group(2).split("+")[0]
        return ("addr", int(m.group(1), 16), base)
    if re.match(r"^(r\d+|sl|fp|ip|lr|pc|sb)\b", operand):
        return ("indirect", None, None)
    return ("none", None, None)

# ---------------------------------------------------------------------------
# Size accounting (size -A)
# ---------------------------------------------------------------------------

def load_sections(elf):
    out = run(["arm-none-eabi-size", "-A", elf])
    sec = {}
    for line in out.splitlines():
        t = line.split()
        if len(t) >= 2 and t[0].startswith(".") and re.fullmatch(r"\d+", t[1]):
            sec[t[0]] = int(t[1])
    return sec

# ---------------------------------------------------------------------------
# Per-ELF driver
# ---------------------------------------------------------------------------

def check_elf(elf, baseline):
    platform = detect_platform(elf)
    loopback = is_loopback(elf)
    lines = []
    fails = warns = 0

    lines.append("=" * 78)
    lines.append("ELF: %s" % elf)
    lines.append("platform=%s  variant=%s" % (platform, "loopback" if loopback else "standard"))
    lines.append("=" * 78)

    syms = load_symbols(elf)
    funcs, starts = parse_disasm(elf)
    entries = hot_list(platform, loopback)

    # --- Check A: hot symbols in RAM ---------------------------------------
    lines.append("[Check A] hot symbols must be RAM-resident")
    ram_roots = []  # symbol addresses confirmed in RAM -> Check B roots
    a_fail = a_warn = 0
    for name, static_ok in entries:
        matches = match_symbols(syms, name)
        ram_hits = [s for s in matches if in_ram(s["addr"])]
        flash_hits = [s for s in matches if in_flash(s["addr"])]
        if flash_hits:
            for s in flash_hits:
                lines.append("  FAIL  %s -> FLASH @0x%08x (%s)" % (name, s["addr"], s["name"]))
                a_fail += 1
            ram_roots += ram_hits  # any RAM aliases still seed Check B
        elif ram_hits:
            ram_roots += ram_hits
        elif static_ok:
            lines.append("  WARN  %s not found (static_ok; likely inlined)" % name)
            a_warn += 1
        else:
            lines.append("  FAIL  %s not found (expected a definition)" % name)
            a_fail += 1
    lines.append("  A summary: %d FAIL, %d WARN, %d RAM roots" % (a_fail, a_warn, len(ram_roots)))
    fails += a_fail
    warns += a_warn

    # --- Check B: no hot function may reach flash --------------------------
    lines.append("[Check B] hot RAM functions must not branch/call into flash")
    b_fail = 0
    indirect_total = 0
    info_whitelist = []
    sorted_addrs = sorted((s["addr"], s["name"]) for s in syms)
    root_starts = []
    for s in ram_roots:
        fs = enclosing_func(starts, funcs, s["addr"])
        if fs is not None:
            root_starts.append(fs)

    def bfs(seed_starts):
        """BFS over RAM function closure; returns (visited, violations,
        indirect count, veneer-to-flash edges)."""
        visited = set()
        violations = []
        veneer_edges = []
        indirect = 0
        queue = list(dict.fromkeys(seed_starts))
        for q in queue:
            visited.add(q)
        head = 0
        while head < len(queue):
            fs = queue[head]
            head += 1
            f = funcs[fs]
            for (addr, mnem, operand) in f["insns"]:
                kind, tgt, tname = parse_target(operand)
                if kind == "indirect":
                    indirect += 1
                    continue
                if kind != "addr":
                    continue
                if f["start"] <= tgt < f["end"]:
                    continue  # internal jump
                if tgt < ROM_HI:
                    continue  # bootrom
                if in_flash(tgt):
                    if tname in WHITELIST:
                        info_whitelist.append((f["name"], addr, tname, tgt))
                    else:
                        violations.append((f["name"], addr, tname, tgt))
                elif in_ram(tgt):
                    ts = enclosing_func(starts, funcs, tgt)
                    if ts is not None and ts not in visited:
                        visited.add(ts)
                        queue.append(ts)
                # peripheral / other ranges: ignore
            # Linker long-call veneers jump via a literal-pool word, which the
            # branch scan cannot see; resolve the literal to find the real
            # target (WARN on flash so a regression is visible, not fatal).
            if f["name"].endswith("_veneer"):
                for (waddr, val) in f["words"]:
                    if in_flash(val):
                        wname = nearest_symbol(sorted_addrs, val)
                        if wname in WHITELIST:
                            info_whitelist.append((f["name"], waddr, wname, val))
                        else:
                            veneer_edges.append((f["name"], waddr, wname, val))
        return visited, violations, indirect, veneer_edges

    _, violations, indirect_total, veneer_edges = bfs(root_starts)
    for (cn, ca, tn, ta) in violations:
        lines.append("  FAIL  %s @0x%08x -> %s @0x%08x (FLASH)" % (cn, ca, tn, ta))
    for (cn, ca, tn, ta) in veneer_edges:
        lines.append("  WARN  %s @0x%08x -> %s @0x%08x (flash via veneer; verify cold-path only)" % (cn, ca, tn, ta))
    for (cn, ca, tn, ta) in info_whitelist:
        lines.append("  INFO  %s @0x%08x -> %s @0x%08x (whitelisted cold path)" % (cn, ca, tn, ta))
    lines.append("  INFO  %d register-indirect branch(es) in the RAM closure (unresolvable)" % indirect_total)
    b_fail = len(violations)
    lines.append("  B summary: %d FAIL, %d veneer WARN" % (b_fail, len(veneer_edges)))
    fails += b_fail
    warns += len(veneer_edges)

    # --- Check B2: core-1 literal-pool flash scan --------------------------
    lines.append("[Check B2] core-1 function bodies must hold no flash-range literals")
    c1_seed = []
    for name in CORE1_ROOTS:
        for s in match_symbols(syms, name):
            if in_ram(s["addr"]):
                fs = enclosing_func(starts, funcs, s["addr"])
                if fs is not None:
                    c1_seed.append(fs)
    c1_visited, _, _, _ = bfs(c1_seed)
    b2_fail = 0
    for fs in sorted(c1_visited):
        f = funcs[fs]
        for (waddr, val) in f["words"]:
            if in_flash(val):
                wname = nearest_symbol(sorted_addrs, val)
                if wname in B2_LITERAL_WHITELIST:
                    lines.append("  INFO  %s @0x%08x -> .word 0x%08x (%s; init-time only, whitelisted)" % (f["name"], waddr, val, wname))
                else:
                    lines.append("  FAIL  %s @0x%08x -> .word 0x%08x (FLASH pointer, near %s)" % (f["name"], waddr, val, wname))
                    b2_fail += 1
    lines.append("  B2 summary: %d core-1 function(s) scanned, %d FAIL" % (len(c1_visited), b2_fail))
    fails += b2_fail

    # --- Check C: size accounting ------------------------------------------
    lines.append("[Check C] size accounting")
    sec = load_sections(elf)
    data = sec.get(".data", 0)
    bss = sec.get(".bss", 0)
    vtab = sec.get(".ram_vector_table", 0)
    heap = sec.get(".heap", 0)
    ram_used = vtab + data + bss + heap
    free_ram = RAM_LEN[platform] - ram_used
    flash_code = sec.get(".text", 0) + sec.get(".rodata", 0) + sec.get(".boot2", 0)
    lines.append("  .data=%d  .bss=%d  vtab=%d  heap=%d  ram_used=%d  free_ram=%d" %
                 (data, bss, vtab, heap, ram_used, free_ram))
    lines.append("  flash_code (.text+.rodata%s)=%d" %
                 ("+.boot2" if ".boot2" in sec else "", flash_code))

    if loopback:
        lines.append("  baseline comparison skipped (loopback variant)")
    elif baseline and platform in baseline:
        b = baseline[platform]
        b_img = b.get("ram_image_bytes", 0)
        b_free = b.get("free_ram_bytes", 0)
        lines.append("  %-22s %12s %12s %12s" % ("metric", "baseline", "current", "delta"))
        lines.append("  %-22s %12d %12d %+12d" % ("ram_image -> .data", b_img, data, data - b_img))
        lines.append("  %-22s %12d %12d %+12d" % ("free_ram", b_free, free_ram, free_ram - b_free))
    else:
        lines.append("  baseline unavailable for platform %s" % platform)

    budget = DATA_BUDGET[platform]
    if data > budget:
        lines.append("  FAIL  .data %d exceeds budget %d" % (data, budget))
        fails += 1
    else:
        lines.append("  .data %d within budget %d" % (data, budget))

    verdict = "FAIL" if fails else ("PASS (with warnings)" if warns else "PASS")
    lines.append("VERDICT: %s  (%d FAIL, %d WARN)" % (verdict, fails, warns))

    return {"elf": elf, "platform": platform, "loopback": loopback,
            "fails": fails, "warns": warns, "data": data, "free_ram": free_ram,
            "text": "\n".join(lines)}

def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2

    here = os.path.dirname(os.path.abspath(__file__))
    baseline = None
    bpath = os.path.join(here, "ram_baseline.json")
    try:
        with open(bpath) as fh:
            baseline = json.load(fh)
    except OSError:
        sys.stderr.write("warning: could not load %s\n" % bpath)

    results = [check_elf(elf, baseline) for elf in argv[1:]]
    for r in results:
        print(r["text"])
        print()

    print("=" * 78)
    print("OVERALL SUMMARY")
    print("%-40s %-8s %5s %5s %10s %10s" % ("elf", "plat", "FAIL", "WARN", ".data", "freeRAM"))
    any_fail = False
    for r in results:
        any_fail = any_fail or r["fails"] > 0
        print("%-40s %-8s %5d %5d %10d %10d" % (
            os.path.basename(os.path.dirname(os.path.dirname(r["elf"]))) or r["elf"],
            r["platform"], r["fails"], r["warns"], r["data"], r["free_ram"]))
    print("=" * 78)
    return 1 if any_fail else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
