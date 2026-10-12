#!/usr/bin/env python3
"""Cycle-level equivalence check for the proposed PIO biphase-mark S/PDIF encoder.
Runs today's 4-instruction program on stamp_frame() words and the proposed
16-instruction encoder on raw words, and compares pin waveforms cycle by cycle.
See Documentation/Features/spdif_pio_bmc_encoder_spec.md."""
import random
import sys

# Lookup table exactly as audio_spdif_setup() builds it: 16 transition bits per
# data byte in the low half, the byte's parity in bit 16.
LUT = []
for i in range(256):
    v = 0x5555
    p = 0
    for j in range(8):
        if i & (1 << j):
            p ^= 1
            v |= 2 << (j * 2)
    LUT.append(v | (p << 16))

PREAMBLE_X = 0b11001001
PREAMBLE_Y = 0b01101001
PREAMBLE_Z = 0b00111001
CHANNEL_STATUS = [0x04, 0x00, 0x00, 0x02, 0x0B]   # 48 kHz


def cs_bit(phase):
    if phase >= 40:
        return 0
    return (CHANNEL_STATUS[phase // 8] >> (phase % 8)) & 1


def update_subframe(l, h, sample):
    # Port of spdif_update_subframe(); the multiply folds the V/U/C parity.
    s0 = LUT[sample & 0xFF]
    s1 = LUT[(sample >> 8) & 0xFF]
    s2 = LUT[(sample >> 16) & 0xFF]
    l = (l & 0xFF) | ((s0 & 0xFFFF) << 8) | ((s1 << 24) & 0xFFFFFFFF)
    ph = h >> 24
    h2 = ((s1 & 0xFFFF) >> 8) | ((s2 & 0xFFFF) << 8)
    p = (s0 >> 16) ^ (s1 >> 16) ^ (s2 >> 16)
    p ^= (((ph & 0x2A) * 0x2A) >> 6) & 1
    h2 |= ((ph & 0x7F) << 24) | (p << 31)
    return l & 0xFFFFFFFF, h2 & 0xFFFFFFFF


def stamp_frame_today(phase, left, right):
    c = cs_bit(phase)
    l0 = PREAMBLE_Z if phase == 0 else PREAMBLE_X
    h0 = 0x55000000 | (c << 29)
    l0, h0 = update_subframe(l0, h0, left)
    l1 = PREAMBLE_Y
    h1 = 0x55000000 | (c << 29)
    l1, h1 = update_subframe(l1, h1, right)
    return [l0, h0, l1, h1]


# Proposed raw words. Word A: lead-low count in [2:0], trail-low count in [5:3],
# each one less than the PIO loop's cycle count. Word B: sample [23:0], V, U, C.
WORD_A_X = 4 | (1 << 3)   # low for 6 cycles, high 2, low 2
WORD_A_Y = 2 | (3 << 3)   # low 4, high 2, low 4
WORD_A_Z = 0 | (5 << 3)   # low 2, high 2, low 6


def stamp_frame_new(phase, left, right):
    c = cs_bit(phase)
    a0 = WORD_A_Z if phase == 0 else WORD_A_X
    b0 = (left & 0xFFFFFF) | (c << 26)
    a1 = WORD_A_Y
    b1 = (right & 0xFFFFFF) | (c << 26)
    return [a0, b0, a1, b1]


class PIO:
    """Subset simulator: OUT/JMP/PULL/NOP, side-set, delay, wrap, shift-right OSR.
    OSRE is shift count >= pull threshold; autopull refills an empty OSR at OUT."""

    def __init__(self, prog, wrap_target, wrap, autopull, thresh, fifo):
        self.prog = prog
        self.wrap_target = wrap_target
        self.wrap = wrap
        self.autopull = autopull
        self.thresh = thresh
        self.fifo = fifo
        self.pc = 0
        self.x = 0
        self.y = 0
        self.osr = 0
        self.cnt = 32           # pio_sm_restart() leaves the OSR empty
        self.pin = 0
        self.trace = []
        self.stalls = 0

    def osre(self):
        return self.cnt >= self.thresh

    def _pull(self):
        if not self.fifo:
            return False
        self.osr = self.fifo.pop(0)
        self.cnt = 0
        return True

    def _out(self, n):
        if self.autopull and self.osre():
            if not self._pull():
                return None
        val = self.osr & ((1 << n) - 1)
        self.osr >>= n
        self.cnt = min(32, self.cnt + n)
        return val

    def step(self):
        ins = self.prog[self.pc]
        if ins.get('side') is not None:
            self.pin = ins['side']
        op = ins['op']
        jumped = False
        stalled = False
        if op == 'nop':
            pass
        elif op == 'pull':
            stalled = not self._pull()
        elif op == 'out':
            v = self._out(ins['n'])
            if v is None:
                stalled = True
            elif ins['dst'] == 'x':
                self.x = v
            elif ins['dst'] == 'y':
                self.y = v
            elif ins['dst'] == 'pc':
                self.pc = v
                jumped = True
            elif ins['dst'] != 'null':
                raise ValueError(ins['dst'])
        elif op == 'jmp':
            cond = ins['cond']
            if cond == 'always':
                take = True
            elif cond == '!x':
                take = self.x == 0
            elif cond == '!y':
                take = self.y == 0
            elif cond == '!osre':
                take = not self.osre()
            elif cond == 'y--':
                # Tests before decrementing, so y=n gives n+1 cycles in a self-loop.
                take = self.y != 0
                self.y = (self.y - 1) & 0xFFFFFFFF
            else:
                raise ValueError(cond)
            if take:
                self.pc = ins['target']
                jumped = True
        else:
            raise ValueError(op)

        self.trace.append(self.pin)
        if stalled:
            self.stalls += 1
            return
        for _ in range(ins.get('delay', 0)):
            self.trace.append(self.pin)
        if not jumped:
            self.pc = self.wrap_target if self.pc == self.wrap else self.pc + 1

    def run(self, cycles):
        while len(self.trace) < cycles:
            self.step()
        del self.trace[cycles:]


def I(op, side=None, delay=0, **kw):
    d = dict(op=op, side=side, delay=delay)
    d.update(kw)
    return d


# Today's audio_spdif.pio: two cycles per half-cell, one transition bit each.
TODAY = [
    I('out', side=0, dst='x', n=1),
    I('jmp', side=0, cond='!x', target=0),
    I('out', side=1, dst='x', n=1),
    I('jmp', side=1, cond='!x', target=2),
]

# Proposed encoder; labels match the spec. Wrap target 0, wrap after TRAIL.
L_BIT, L_LOW2, COMMON, LEAD, TRAIL, H_BIT, H_HIGH2 = 0, 2, 5, 7, 10, 11, 13
NEW = [
    I('out', side=1, dst='x', n=1),                         # 0  L_bit
    I('jmp', side=1, cond='!x', target=H_HIGH2),            # 1
    I('jmp', side=0, delay=1, cond='!osre', target=L_BIT),  # 2  L_low2
    I('nop', side=1, delay=1),                              # 3  P=1 half 1
    I('pull', side=0, delay=1),                             # 4  P=1 half 2, word A (start entry)
    I('nop', side=1, delay=5),                              # 5  common: preamble high x6
    I('out', side=0, dst='y', n=3),                         # 6
    I('jmp', side=0, cond='y--', target=LEAD),              # 7  lead
    I('out', side=1, dst='y', n=3),                         # 8
    I('pull', side=1),                                      # 9  word B
    I('jmp', side=0, cond='y--', target=TRAIL),             # 10 trail, then wrap
    I('out', side=0, dst='x', n=1),                         # 11 H_bit
    I('jmp', side=0, cond='!x', target=L_LOW2),             # 12
    I('jmp', side=1, delay=1, cond='!osre', target=H_BIT),  # 13 H_high2
    I('pull', side=0, delay=2),                             # 14 P=0, word A
    I('jmp', side=0, cond='always', target=COMMON),         # 15
]
NEW_ENTRY = 4
assert len(NEW) == 16


def run_compare(frames, seed):
    rng = random.Random(seed)
    # Two spare frames: the new program pulls the next control word before the
    # subframe ends, so a stream cut exactly at the window would stall it.
    samples = [(rng.getrandbits(32), rng.getrandbits(32)) for _ in range(frames + 2)]
    samples[0] = (0, 0)
    samples[1] = (0xFFFFFF, 0x800000)
    samples[2] = (0xFFFFFFFF, 0x7FFFFF)
    fifo_today, fifo_new = [], []
    for f, (l, r) in enumerate(samples):
        fifo_today += stamp_frame_today(f % 192, l, r)
        fifo_new += stamp_frame_new(f % 192, l, r)

    cycles = frames * 256
    a = PIO(TODAY, wrap_target=0, wrap=3, autopull=True, thresh=32, fifo=fifo_today)
    a.run(cycles)
    b = PIO(NEW, wrap_target=L_BIT, wrap=TRAIL, autopull=False, thresh=27, fifo=fifo_new)
    b.pc = NEW_ENTRY
    b.run(cycles)
    assert a.stalls == 0 and b.stalls == 0, (a.stalls, b.stalls)
    mism = [i for i in range(cycles) if a.trace[i] != b.trace[i]]
    return a, b, mism


def main():
    frames = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    for seed in range(5):
        a, b, mism = run_compare(frames, seed)
        if mism:
            i = mism[0]
            print(f"seed {seed}: MISMATCH at cycle {i} (frame {i // 256}, cycle {i % 128} of subframe)")
            lo = max(0, i - 20)
            print(" today:", ''.join(map(str, a.trace[lo:i + 40])))
            print(" new  :", ''.join(map(str, b.trace[lo:i + 40])))
            sys.exit(1)
        print(f"seed {seed}: identical waveform over {frames} frames ({frames * 256} cycles)")
    a, b, _ = run_compare(4, 0)
    print("frame 2 subframe A, today:", ''.join(map(str, a.trace[512:640])))
    print("frame 2 subframe A, new  :", ''.join(map(str, b.trace[512:640])))
    print("start lead-in, today:", a.trace[:8], " new:", b.trace[:8])


if __name__ == '__main__':
    main()
