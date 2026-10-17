#!/usr/bin/env python3
"""Cycle-level jitter-margin check for the pico_spdif_rx decoder PIO programs.
Models the 21-instruction decoder (wait/jmp pin/in/push, 2-cycle input
synchroniser, 2.5 fractional PIO clock) against a BMC-encoded S/PDIF stream and
sweeps edge distortion and random jitter until decoding fails. Usage:
  spdif_pio_rx_sim.py            # margin table for every rate/program pair
  spdif_pio_rx_sim.py 44100 22 10 [frames]   # one program, verbose"""
import random
import sys

SYS_HZ = 307_200_000
PIO_DIV = 2.5                      # PIO ticks land on sys cycles 0, 2, 5, 7, 10 ...
NS_PER_SYS = 1e9 / SYS_HZ

PREAMBLE = {'B': '11101000', 'M': '11100010', 'W': '11100100'}
SYNC_CODE = {'B': 0b1111, 'M': 0b1011, 'W': 0b0111}


def build_program(cy, lp):
    """Instruction list mirroring spdif_rx_*.pio; index 0 is entry_point,
    index 1 is wrap_target, index 11 wraps."""
    return [
        ('wait', 0, 0),                 # 0  entry_point
        ('wait', 1, cy - 1 + lp),       # 1  wait1 (wrap_target)
        ('jmp_pin', 5, 0),              # 2  symbol_1x
        ('in', 1, 1),                   # 3  emit 1 (symbol 10)
        ('jmp', 1, 0),                  # 4
        ('nop', 0, cy - 2),             # 5  symbol_11x
        ('jmp_pin', 14, 0),             # 6
        ('in', 0, 1),                   # 7  emit 0 (symbol 110)
        ('jmp', 10, cy - 3),            # 8
        ('wait', 0, cy - 1 + lp),       # 9  wait0
        ('jmp_pin', 12, 0),             # 10 symbol_0x
        ('in', 0, 1),                   # 11 emit 0 (symbol 00), .wrap
        ('in', 1, 1),                   # 12 symbol_01: emit 1
        ('jmp', 9, 0),                  # 13
        ('push', 0, 0),                 # 14 sync1110
        ('in', 3, 2),                   # 15 emit 11
        ('wait', 0, cy - 1 + lp),       # 16
        ('jmp_pin', 19, cy - 2),        # 17
        ('jmp', 10, 0),                 # 18
        ('in', 3, 2),                   # 19 sync1xxx: emit 11
        ('jmp', 0, 0),                  # 20
    ]


def make_stream(fs, frames, seed):
    """Transition times (sys cycles) and expected PIO words for `frames`
    frames. Each subframe ends at level 0 (even parity), so every preamble
    appears in its E8/E2/E4 form and the decoder's single polarity suffices."""
    rnd = random.Random(seed)
    sym = SYS_HZ / (fs * 128.0)
    t = 0.0
    level = 0
    edges = []
    expected = []
    for f in range(frames):
        for ch in range(2):
            kind = 'W' if ch else ('B' if f % 192 == 0 else 'M')
            # Even parity returns every subframe to level 0, which is what lets
            # the decoder handle a single preamble polarity.
            assert level == 0, 'subframe did not end at level 0'
            audio = rnd.choice([0, 0xFFFFFF, 0x800000, 0x7FFFFF, rnd.getrandbits(24)])
            c = rnd.getrandbits(1)
            bits = [(audio >> i) & 1 for i in range(24)] + [0, 0, c]
            bits.append(sum(bits) & 1)          # P: even parity over bits 4..31
            for s in PREAMBLE[kind]:
                s = int(s)
                if s != level:
                    edges.append(t)
                    level = s
                t += sym
            for b in bits:                      # one bit cell = two symbols
                edges.append(t)                 # cell-start transition
                level ^= 1
                if b:
                    edges.append(t + sym)       # mid-cell transition for a 1
                    level ^= 1
                t += 2 * sym
            word = SYNC_CODE[kind]
            for i, b in enumerate(bits):
                word |= b << (4 + i)
            expected.append(word)
    return edges, expected, t


def perturb(edges, pwd_sys, jitter_sys, seed):
    """Shift every falling edge by pwd_sys (pulse-width distortion) and every
    edge by Gaussian jitter. Edges alternate polarity starting with a rise."""
    rnd = random.Random(seed)
    out = []
    for i, e in enumerate(edges):
        if i & 1:
            e += pwd_sys
        if jitter_sys:
            e += rnd.gauss(0.0, jitter_sys)
        out.append(e)
    return out


def run(prog, edges, t_end):
    """Execute the program over the waveform; return the pushed words."""
    nedge = len(edges)
    ei = 0
    level_now = 0

    def level_at(t):
        nonlocal ei, level_now
        while ei < nedge and edges[ei] <= t:
            level_now ^= 1
            ei += 1
        return level_now

    pc = 0
    stall = 0
    isr = 0
    pushed = []
    synced = [0, 0]                 # two-stage input synchroniser
    k = 0
    while True:
        tick_t = (k * 5) // 2       # 2.5 sys cycles per tick, integer pattern 0,2,5,7,...
        if tick_t > t_end:
            break
        pin = synced[0]
        synced = [synced[1], level_at(tick_t)]
        k += 1
        if stall:
            stall -= 1
            continue
        op, a, d = prog[pc]
        nxt = 1 if pc == 11 else pc + 1     # .wrap after index 11
        delay = d
        if op == 'wait':
            if pin != a:
                continue            # stalls do not consume delay cycles
        elif op == 'jmp_pin':
            if pin:
                nxt = a
        elif op == 'jmp':
            nxt = a
        elif op == 'in':            # ('in', value, nbits): shift right into ISR
            isr = ((isr >> d) | ((a & ((1 << d) - 1)) << (32 - d))) & 0xFFFFFFFF
            delay = 0
        elif op == 'push':
            pushed.append(isr)
            isr = 0
        stall = delay
        pc = nxt
    return pushed


def decode_ok(prog, fs, frames, pwd_ns, jitter_ns, seed):
    edges, expected, t_end = make_stream(fs, frames, seed)
    edges = perturb(edges, pwd_ns / NS_PER_SYS, jitter_ns / NS_PER_SYS, seed + 1)
    words = run(prog, edges, t_end)
    # The first push is the partial word before the first preamble; the last
    # expected word is never pushed (no preamble follows it).
    got = words[1:]
    want = expected[:len(got)]
    if len(got) < len(expected) - 2:
        return False, 'only %d of %d words' % (len(got), len(expected))
    bad = sum(1 for g, w in zip(got, want) if g != w)
    return bad == 0, '%d bad of %d' % (bad, len(got))


def sweep(prog, fs, frames, seed):
    """Largest |pulse-width distortion| in each direction and largest jitter
    sigma (ns) that still decode every word."""
    def edge_limit(sign):
        lo, hi = 0.0, 120.0
        for _ in range(14):
            mid = (lo + hi) / 2
            ok, _ = decode_ok(prog, fs, frames, sign * mid, 0.0, seed)
            lo, hi = (mid, hi) if ok else (lo, mid)
        return lo
    late = edge_limit(+1)
    early = edge_limit(-1)
    lo, hi = 0.0, 40.0
    for _ in range(12):
        mid = (lo + hi) / 2
        ok, _ = decode_ok(prog, fs, frames, 0.0, mid, seed)
        lo, hi = (mid, hi) if ok else (lo, mid)
    return late, early, lo


def main():
    if len(sys.argv) >= 4:
        fs, cy, lp = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
        frames = int(sys.argv[4]) if len(sys.argv) > 4 else 400
        prog = build_program(cy, lp)
        ok, msg = decode_ok(prog, fs, frames, 0.0, 0.0, 1)
        print('clean decode: %s (%s)' % ('OK' if ok else 'FAIL', msg))
        late, early, jit = sweep(prog, fs, frames, 1)
        print('falling edge late  up to %5.1f ns' % late)
        print('falling edge early up to %5.1f ns' % early)
        print('random jitter sigma up to %4.1f ns' % jit)
        return
    cases = [
        (44100, 20, 10, 'old (shared 48 kHz program)'),
        (44100, 22, 10, 'new'),
        (48000, 20, 10, ''),
        (88200, 10, 5, 'old (shared 96 kHz program)'),
        (88200, 11, 5, 'new'),
        (96000, 10, 5, ''),
    ]
    print('%7s %3s %3s  %-30s %8s %8s %8s' %
          ('rate', 'cy', 'lp', 'program', 'late ns', 'early ns', 'jit ns'))
    for fs, cy, lp, note in cases:
        prog = build_program(cy, lp)
        ok, msg = decode_ok(prog, fs, 400, 0.0, 0.0, 1)
        if not ok:
            print('%7d %3d %3d  %-30s CLEAN DECODE FAILED: %s' % (fs, cy, lp, note, msg))
            continue
        late, early, jit = sweep(prog, fs, 400, 1)
        print('%7d %3d %3d  %-30s %8.1f %8.1f %8.1f' % (fs, cy, lp, note, late, early, jit))


if __name__ == '__main__':
    main()
