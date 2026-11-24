#!/usr/bin/env python3
"""ui_fontgen.py; generates the front-panel UI fonts for firmware/DSPi/ui/.

  font_block.h  "Block": squared, uniform-stroke 5x7 capitals (plus "d"),
                drawn crisp at 1x.
  font_round.h  "Geometric rounded" (Gugi-inspired): a round pen along the
                Block letterforms, every right-angle corner a quarter circle,
                square corners kept on stems. Pre-rendered at 2x, 3x and 4x.

Edit the glyphs in BLOCK (7 rows of 5, '#' = ink) or the stroke weights in WF,
then run from the repo root:

    python tools/ui_fontgen.py

Pure Python, no dependencies. GPL-3.0 with the rest of the firmware.
"""
import math
import os

BLOCK = {
    '0': "#####|#...#|#..##|#.#.#|##..#|#...#|#####",
    '1': "..#..|.##..|..#..|..#..|..#..|..#..|.###.",
    '2': "#####|....#|....#|#####|#....|#....|#####",
    '3': "#####|....#|....#|.####|....#|....#|#####",
    '4': "#...#|#...#|#...#|#####|....#|....#|....#",
    '5': "#####|#....|#....|#####|....#|....#|#####",
    '6': "#####|#....|#....|#####|#...#|#...#|#####",
    '7': "#####|....#|....#|....#|....#|....#|....#",
    '8': "#####|#...#|#...#|#####|#...#|#...#|#####",
    '9': "#####|#...#|#...#|#####|....#|....#|#####",
    'A': "#####|#...#|#...#|#####|#...#|#...#|#...#",
    'B': "####.|#...#|#...#|####.|#...#|#...#|####.",
    'C': "#####|#....|#....|#....|#....|#....|#####",
    'D': "####.|#...#|#...#|#...#|#...#|#...#|####.",
    'E': "#####|#....|#....|####.|#....|#....|#####",
    'F': "#####|#....|#....|####.|#....|#....|#....",
    'G': "#####|#....|#....|#..##|#...#|#...#|#####",
    'H': "#...#|#...#|#...#|#####|#...#|#...#|#...#",
    'I': "#####|..#..|..#..|..#..|..#..|..#..|#####",
    'J': "....#|....#|....#|....#|....#|#...#|#####",
    'K': "#...#|#..#.|#.#..|###..|#..#.|#...#|#...#",
    'L': "#....|#....|#....|#....|#....|#....|#####",
    'M': "#...#|##.##|#.#.#|#.#.#|#...#|#...#|#...#",
    'N': "#...#|##..#|#.#.#|#..##|#...#|#...#|#...#",
    'O': "#####|#...#|#...#|#...#|#...#|#...#|#####",
    'P': "#####|#...#|#...#|#####|#....|#....|#....",
    'Q': "#####|#...#|#...#|#...#|#.#.#|#..#.|###.#",
    'R': "#####|#...#|#...#|#####|#..#.|#...#|#...#",
    'S': "#####|#....|#....|#####|....#|....#|#####",
    'T': "#####|..#..|..#..|..#..|..#..|..#..|..#..",
    'U': "#...#|#...#|#...#|#...#|#...#|#...#|#####",
    'V': "#...#|#...#|#...#|#...#|#...#|.#.#.|..#..",
    'W': "#...#|#...#|#...#|#.#.#|#.#.#|##.##|#...#",
    'X': "#...#|#...#|.#.#.|..#..|.#.#.|#...#|#...#",
    'Y': "#...#|#...#|#...#|#####|..#..|..#..|..#..",
    'Z': "#####|....#|...#.|..#..|.#...|#....|#####",
    '-': ".....|.....|.....|#####|.....|.....|.....",
    '+': ".....|..#..|..#..|#####|..#..|..#..|.....",
    '.': ".....|.....|.....|.....|.....|.....|..#..",
    '%': "##..#|##.#.|...#.|..#..|.#...|.#.##|#..##",
    '<': "...#.|..#..|.#...|#....|.#...|..#..|...#.",
    ':': ".....|..#..|.....|.....|.....|..#..|.....",
    ' ': ".....|.....|.....|.....|.....|.....|.....",
    # The one lowercase glyph, for "dB".
    'd': "....#|....#|#####|#...#|#...#|#...#|#####",
}


SS = 8

# Gugi keeps a straight, square-cornered stem on spine letters.
SPINE = set("BDEFHKLMNPR")      # square corners on the left stem
STEM_RIGHT = set("d")            # square corners on the right stem
OVERRIDE = {
    'B': "#####|#...#|#...#|####.|#...#|#...#|#####",
    'D': "#####|#...#|#...#|#...#|#...#|#...#|#####",
}


def skeleton(ch):
    key = ch if ch in BLOCK and ch.islower() else ch.upper()
    rows = OVERRIDE.get(key, BLOCK.get(key, BLOCK[' '])).split('|')
    on = lambda i, r: 0 <= r < 7 and 0 <= i < 5 and rows[r][i] == '#'
    nodes = [(i, r) for r in range(7) for i in range(5) if on(i, r)]
    edges = set()
    for (i, r) in nodes:
        for di, dr in ((1, 0), (0, 1)):
            if on(i + di, r + dr):
                edges.add(((i, r), (i + di, r + dr)))
        for di, dr in ((1, 1), (-1, 1)):          # diagonals not covered by an L
            if on(i + di, r + dr) and not on(i + di, r) and not on(i, r + dr):
                edges.add(((i, r), (i + di, r + dr)))
    return nodes, edges


def render(ch, s, wf, corner=True):
    nodes, edges = skeleton(ch)
    key = ch if ch in BLOCK and ch.islower() else ch.upper()
    sharp_col = 0 if key in SPINE else (4 if key in STEM_RIGHT else -1)
    cell = s * SS
    R = wf * cell / 2
    M = SS * 2                                  # 2 px margin each side, supersampled
    Wd, Hd = 5 * cell + 2 * M, 7 * cell + 2 * M
    hi = [bytearray(Wd) for _ in range(Hd)]
    P = lambda n: (M + (n[0] + 0.5) * cell, M + (n[1] + 0.5) * cell)

    def disc(cx, cy):
        for y in range(max(0, int(cy - R)), min(Hd, int(cy + R) + 2)):
            dy = y + .5 - cy
            if abs(dy) > R:
                continue
            dx = math.sqrt(R * R - dy * dy)
            for x in range(max(0, int(cx - dx + .5)), min(Wd, int(cx + dx + .5))):
                hi[y][x] = 1

    def seg(a, b):
        (x0, y0), (x1, y1) = a, b
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 2) + 1)
        for k in range(n + 1):
            t = k / n
            disc(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

    deg = {}
    for a, b in edges:
        deg.setdefault(a, []).append(b)
        deg.setdefault(b, []).append(a)
    arced = set()
    if corner:
        for c, nb in deg.items():
            if len(nb) != 2 or c[0] == sharp_col:
                continue
            (a, b) = nb
            va, vb = (a[0] - c[0], a[1] - c[1]), (b[0] - c[0], b[1] - c[1])
            if abs(va[0]) + abs(va[1]) == 1 and abs(vb[0]) + abs(vb[1]) == 1 and va[0] * vb[0] + va[1] * vb[1] == 0:
                arced.add(c)
                ctr = (a[0] + b[0] - c[0], a[1] + b[1] - c[1])
                cx, cy = P(ctr)
                pa, pb = P(a), P(b)
                a0 = math.atan2(pa[1] - cy, pa[0] - cx)
                a1 = math.atan2(pb[1] - cy, pb[0] - cx)
                d = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
                n = 24
                for k in range(n + 1):
                    t = a0 + d * k / n
                    disc(cx + cell * math.cos(t), cy + cell * math.sin(t))
    for a, b in edges:
        if a in arced or b in arced:
            continue
        seg(P(a), P(b))
    for n in nodes:
        if n not in arced and not deg.get(n):
            disc(*P(n))                          # isolated dot
    for n in nodes:
        if n not in arced and n in deg:
            disc(*P(n))
    if sharp_col >= 0:
        # Square pen on the stem's corners: a crisp corner where the stem
        # meets a bar, as Gugi's B/D/E/F/... have.
        for c, nb in deg.items():
            if c[0] != sharp_col or len(nb) != 2:
                continue
            cx, cy = P(c)
            for y in range(max(0, int(cy - R)), min(Hd, int(cy + R))):
                for x in range(max(0, int(cx - R)), min(Wd, int(cx + R))):
                    hi[y][x] = 1
    w, h = 5 * s + 4, 7 * s + 4
    out = []
    for y in range(h):
        row = 0
        for x in range(w):
            cnt = 0
            for a in range(SS):
                line = hi[y * SS + a]
                cnt += sum(line[x * SS:(x + 1) * SS])
            if cnt * 2 >= SS * SS:
                row |= 1 << x
        out.append(row)
    return out, w, h



WF = {2: 1.15, 3: 1.3, 4: 1.3}   # stroke width per scale, as a fraction of a grid cell

CHARS = sorted(BLOCK.keys())     # glyph order shared by both headers


def block_header():
    lines = []
    for ch in CHARS:
        rows = BLOCK[ch].split('|')
        assert len(rows) == 7 and all(len(r) == 5 for r in rows), ch
        cols = [sum(1 << r for r in range(7) if rows[r][c] == '#') for c in range(5)]
        lines.append("    { '%s', { %s } }," % (ch, ", ".join("0x%02X" % v for v in cols)))
    return """/* font_block.h; "Block" display font: squared, uniform-stroke capitals,
 * digits, the punctuation the UI uses and a lowercase "d" (for dB). 5x7,
 * column-major like font5x7.h (bit 0 = top row). Other lowercase maps onto
 * the capitals; anything not listed falls back to font5x7.
 * Generated by tools/ui_fontgen.py. GPL-3.0 with the rest of the firmware. */
#ifndef FONT_BLOCK_H
#define FONT_BLOCK_H
#include <stdint.h>

typedef struct { char ch; uint8_t col[5]; } BlockGlyph;

static const BlockGlyph font_block[] = {
""" + "\n".join(lines) + """
};

#define FONT_BLOCK_COUNT (sizeof font_block / sizeof font_block[0])

#endif
"""


def round_table(sc, ctype, name):
    lines = []
    w = h = 0
    for ch in CHARS:
        if ch == ' ':
            w, h = 5 * sc + 4, 7 * sc + 4
            g = [0] * h
        else:
            g, w, h = render(ch, sc, WF[sc])
        lines.append("    { " + ", ".join("0x%X" % v for v in g) + " },  // '%s'" % ch)
    body = "static const %s %s[%d][%d] = {\n%s\n};\n" % (ctype, name, len(CHARS), h, "\n".join(lines))
    return w, h, body


def round_header():
    parts, defs, asserts = [], [], []
    for sc, ctype in ((2, "uint16_t"), (3, "uint32_t"), (4, "uint32_t")):
        name = "font_round%d" % sc
        w, h, body = round_table(sc, ctype, name)
        defs.append("#define FONT_ROUND%d_W %d\n#define FONT_ROUND%d_H %d" % (sc, w, sc, h))
        parts.append(body)
        asserts.append('_Static_assert(sizeof %s / sizeof %s[0] == FONT_BLOCK_COUNT, '
                       '"glyph order must match font_block");' % (name, name))
    return """/* font_round.h; "Geometric rounded" display font (Gugi-inspired): a round
 * pen along the Block letterforms (font_block.h, same glyph order), with
 * every right-angle corner drawn as a quarter circle and square corners kept
 * on stems (B D E F H K L M N P R, and d). Pre-rendered for the scales the
 * UI uses (2x, 3x, 4x); rows are bit masks, bit 0 = leftmost pixel, and each
 * bitmap has a 2-pixel margin on every side (draw at x - 2, y - 2).
 * Generated by tools/ui_fontgen.py. GPL-3.0 with the rest of the firmware. */
#ifndef FONT_ROUND_H
#define FONT_ROUND_H
#include <stdint.h>
#include "font_block.h"

#define FONT_ROUND_MARGIN 2
""" + "\n".join(defs) + "\n\n" + "\n".join(parts) + "\n" + "\n".join(asserts) + """

#endif
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ui = os.path.join(here, "..", "firmware", "DSPi", "ui")
    for name, text in (("font_block.h", block_header()), ("font_round.h", round_header())):
        path = os.path.normpath(os.path.join(ui, name))
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("wrote", path)


if __name__ == "__main__":
    main()
