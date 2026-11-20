#!/usr/bin/env python3
"""Cycle-level check of the I2S external-clock output PIO program.

Models the retired 26-instruction audio_i2s_dataout_extclk program and the
current 17-instruction one (2-flop pad synchroniser, autopull at 32, shift
left), plus the firmware's C-side OSR priming, and checks that:
  1. priming consumes exactly two FIFO words and leaves one OSR bit, under
     either autopull-on-exec behaviour;
  2. on clean clocks both programs put the same samples on the wire
     (several BCK periods, with and without one cycle of LRCLK skew);
  3. every BCK/LRCLK glitch the old program flags, the new one flags too,
     and any glitch neither flags leaves identical wire output.
Usage: i2s_extclk_pio_sim.py   (exit status 1 on any failure)"""
import random
import sys

BCK, LR = 0, 1


def I(op, **k):
    k['op'] = op
    return k


def W(pol, g):
    return I('wait', pol=pol, gpio=g)


# Retired program: two in-program discards, re-frames itself after a slip.
OLD = [I('out', dst='null', n=32), I('out', dst='null', n=32),
       W(1, LR), W(0, LR), W(0, BCK),                            # 2 reframe
       I('set', dst='x', v=30),                                  # 5 wrap_target
       W(1, BCK), W(0, BCK), I('out', dst='pins', n=1),          # 6 bitpair
       W(1, BCK), W(0, BCK), I('out', dst='pins', n=1),
       I('jmp', cond='x--', t=6),
       W(1, BCK), W(0, BCK), I('out', dst='pins', n=1),          # bit 63
       W(1, BCK), I('jmp', cond='pin', t=19), I('jmp', cond='always', t=23),
       W(0, BCK), I('out', dst='pins', n=1), W(1, BCK),          # 19 bit64
       I('jmp', cond='pin', t=24),                               # 22 wrap
       I('out', dst='null', n=32), I('irq'), I('jmp', cond='always', t=2)]
OLD_WRAP = (5, 22)

# Current program, index-for-index with audio_i2s_dataout_extclk.pio.
NEW = [W(1, LR), W(0, LR),                                       # 0 entry_point
       W(0, BCK), I('out', dst='pins', n=1), W(1, BCK),          # 2 top
       I('jmp', cond='pin', t=14),                               # mid R32 low
       I('set', dst='y', v=1), I('set', dst='x', v=31),
       W(0, BCK), I('out', dst='pins', n=1), W(1, BCK),          # 8 bit
       I('jmp', cond='x--', t=8),
       I('jmp', cond='y--', t=15),
       I('jmp', cond='pin', t=2),                                # mid R31 high
       I('irq'),                                                 # 14 slip (wraps to itself)
       I('set', dst='x', v=30), I('jmp', cond='always', t=8)]    # 15 second
NEW_WRAP = (14, 14)
assert len(NEW) == 17 and len(OLD) == 26


class SM:
    def __init__(self, prog, wrap, fifo, restart_cnt=32):
        self.p, (self.wt, self.w) = prog, wrap
        self.fifo, self.fi = fifo, 0
        self.pc = 0
        self.x = self.y = 0
        self.osr = 0xDEADBEEF          # stale contents survive SM_RESTART
        self.cnt = restart_cnt         # output shift count; 32 = empty
        self.pin = 0
        self.irq = 0

    def pulled(self):
        return self.fi

    def _pull(self):
        if self.fi >= len(self.fifo):
            return False
        self.osr, self.cnt = self.fifo[self.fi], 0
        self.fi += 1
        return True

    def out(self, n, autopull_after=True):
        if self.cnt >= 32 and not self._pull():
            return None                # stall: FIFO empty
        v = (self.osr >> (32 - n)) & ((1 << n) - 1)
        self.osr = (self.osr << n) & 0xFFFFFFFF
        self.cnt += n
        if autopull_after and self.cnt >= 32:
            self._pull()
        return v

    def pull_exec(self):
        # Autopull on: PULL is a no-op while the OSR still holds data.
        if self.cnt < 32:
            return
        assert self._pull(), "priming pull would stall"

    def step(self, g):
        i = self.p[self.pc]
        op = i['op']
        nxt = self.wt if self.pc == self.w else self.pc + 1
        if op == 'wait':
            if g[i['gpio']] != i['pol']:
                return False
        elif op == 'out':
            v = self.out(i['n'])
            if v is None:
                return False
            if i['dst'] == 'pins':
                self.pin = v & 1
        elif op == 'set':
            setattr(self, i['dst'], i['v'])
        elif op == 'irq':
            self.irq += 1
        elif op == 'jmp':
            c = i['cond']
            take = True
            if c == 'pin':
                take = g[LR] == 1
            elif c == 'x--':
                take = self.x != 0
                self.x = (self.x - 1) & 0xFFFFFFFF
            elif c == 'y--':
                take = self.y != 0
                self.y = (self.y - 1) & 0xFFFFFFFF
            if take:
                nxt = i['t']
        self.pc = nxt
        return True


def prime(sm, autopull_on_exec):
    """Firmware priming (audio_i2s_multi.c i2s_extclk_prime): SM disabled,
    FIFO prefilled, after SM_RESTART."""
    sm.pull_exec()
    sm.out(32, autopull_after=autopull_on_exec)
    sm.pull_exec()
    sm.out(31, autopull_after=autopull_on_exec)


def clocks(frames, per, glitch=None, short=None):
    """Raw (bck, lr) per sys cycle; LRCLK changes on BCK falling edges.
    glitch=(cell, kind): runt/drop BCK pulse, lrshort/lrlong right half.
    short=(frame, halfcells): one LRCLK period of 2*halfcells cells."""
    out = []
    lr, lrcount, half, f = 1, 16, per // 2, 0
    for c in range(frames * 64):
        lrcount += 1
        lim = 32
        if glitch and c // 64 == glitch[0] // 64 and lr == 1:
            lim = {'lrshort': 29, 'lrlong': 35}.get(glitch[1], 32)
        if short and f == short[0]:
            lim = short[1]
        if lrcount >= lim:
            lr ^= 1
            lrcount = 0
            if lr == 0:
                f += 1
        seg = [(0, lr)] * half + [(1, lr)] * (per - half)
        if glitch and c == glitch[0]:
            if glitch[1] == 'runt':
                seg[2:4] = [(1, lr)] * 2
            elif glitch[1] == 'drop':
                seg = [(0, lr)] * per
        out += seg
    return out


def run(prog, wrap, words, clk, skew=0, primed=None):
    """Fast-forwards stalled waits to the next synchronised input change."""
    n = len(clk)
    g = [(clk[max(t - 2, 0)][0], clk[max(t - 2 - skew, 0)][1]) for t in range(n)]
    nxt_chg = [[n] * (n + 1), [n] * (n + 1)]
    for k in (0, 1):
        arr = nxt_chg[k]
        for t in range(n - 2, -1, -1):
            arr[t] = t + 1 if g[t + 1][k] != g[t][k] else arr[t + 1]
    sm = SM(prog, wrap, words)
    if primed is not None:
        prime(sm, primed)
    pin_log = [(0, sm.pin)]
    t = 0
    while t < n:
        pc = sm.pc
        if sm.step(g[t]):
            if sm.pin != pin_log[-1][1]:
                pin_log.append((t + 1, sm.pin))
            t += 1
        else:
            i = sm.p[pc]
            t = nxt_chg[i['gpio']][t] if i['op'] == 'wait' else n
    # Receiver samples data on raw BCK rising edges.
    bits, j = [], 0
    for t in range(1, n):
        if clk[t][0] == 1 and clk[t - 1][0] == 0:
            while j + 1 < len(pin_log) and pin_log[j + 1][0] <= t:
                j += 1
            bits.append((pin_log[j][1], clk[t][1]))
    return sm, bits


def decode(bits):
    """Standard I2S: the cell where LRCLK reads newly low is R32."""
    frames, cur, plr = [], None, bits[0][1]
    for d, lr in bits:
        if plr == 1 and lr == 0:
            if cur is not None:
                cur.append(d)
                if len(cur) == 64:
                    frames.append((int(''.join(map(str, cur[:32])), 2),
                                   int(''.join(map(str, cur[32:])), 2)))
                else:
                    frames.append(None)
            cur = []
        elif cur is not None:
            cur.append(d)
        plr = lr
    return frames


def main():
    fails = 0
    rng = random.Random(1)
    N = 48
    words = [rng.getrandbits(32) for _ in range(2 * (N + 8))]
    src = [(words[2 * i], words[2 * i + 1]) for i in range(N + 8)]

    for ape in (False, True):
        sm = SM(NEW, NEW_WRAP, words)
        prime(sm, ape)
        ok = sm.pulled() == 2 and sm.cnt == 31 and sm.osr >> 31 == words[1] & 1
        print(f"priming, autopull-on-exec={ape}: words consumed={sm.pulled()} "
              f"osr bits left={32 - sm.cnt} {'OK' if ok else 'FAIL'}")
        fails += not ok
    sm = SM(NEW, NEW_WRAP, words, restart_cnt=0)
    prime(sm, True)
    print(f"priming if SM_RESTART left the OSR full: consumed={sm.pulled()} "
          f"(firmware read-address check expects 2 and panics otherwise)")

    def both(clk, skew=0):
        r = []
        for prog, wrap, primed in ((OLD, OLD_WRAP, None), (NEW, NEW_WRAP, True)):
            sm, bits = run(prog, wrap, words, clk, skew, primed)
            r.append((sm.irq > 0, decode(bits)))
        return r

    for per in (12, 16, 50, 100):
        for skew in (0, 1):
            (fo, a), (fn, b) = both(clocks(N, per), skew)
            # SM starts before an LRCLK fall, so the first frame window carries sample 1.
            ok = (not fo and not fn and a == b and len(a) > N - 4
                  and all(f == src[i + 1] for i, f in enumerate(a)))
            print(f"clean BCK={per} cyc skew={skew}: first sample={src.index(b[0])} "
                  f"frames={len(b)} identical={a == b} {'OK' if ok else 'FAIL'}")
            fails += not ok

    cases = missed = extra = differ = 0
    G = 20 * 64
    matrix = [dict(glitch=(G + c, k)) for k in ('runt', 'drop', 'lrshort', 'lrlong')
              for c in range(64)]
    matrix += [dict(short=(20, hc)) for hc in range(1, 32)]
    for m in matrix:
        (fo, a), (fn, b) = both(clocks(N, 16, **m))
        cases += 1
        if fo and not fn:
            missed += 1
            print(f"  MISSED {m}")
        elif fn and not fo:
            extra += 1
            print(f"  flagged only by new {m}")
        elif not fo and not fn and a != b:
            differ += 1
            print(f"  UNFLAGGED OUTPUT DIFFERS {m}")
    print(f"{cases} glitch cases: missed by new={missed}, flagged only by new={extra}, "
          f"unflagged but different output={differ}")
    fails += missed + differ
    print("PASS" if not fails else f"FAIL ({fails})")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
