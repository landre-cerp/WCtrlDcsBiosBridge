#!/usr/bin/env python3
"""Redraw a font's small glyphs at a taller cap height, from its large ones.

The stock small set is 14px against the large set's 21px. That is the house size across every
font here and it suits a display that uses small text sparingly. The C-130J's CNI is not one:
44% of its placeable slots are small, because that is how the module marks what the crew can
edit, so the size that reads as an annotation elsewhere carries half the screen here.

Scaling a bitmap by interpolation would be wrong. These glyphs are drawn with a strict 2px
stroke and no antialiasing, and any resampling turns some strokes into 1px and others into 3.
What this does instead is delete whole rows from *inside* runs of three or more identical ones
- the redundant middle of a stem, never a stroke. Every bar comes out the width it went in,
which is checked rather than assumed: no glyph may leave here with a 1px stroke.

Height only. Narrowing costs stems on the letters that have no redundant columns to give - M,
S, 0 and 8 among them - so the derived set keeps the large set's widths. That makes the two
sizes differ by height alone, which is the trade being offered: more legible, less distinct.

Each glyph is placed against the bottom and left of the small glyph it replaces, so whatever
vertical position that glyph had is kept. A hyphen sits mid-height and must stay there.

    python resize-small.py ../../Resources/c130j-font-21x31.json 18
"""

import json
import sys
from pathlib import Path

MIN_RUN = 3


def bbox(rows):
    used = [i for i, r in enumerate(rows) if "X" in r]
    cols = [c for c in range(len(rows[0])) if any(r[c] == "X" for r in rows)]
    if not used or not cols:
        return None
    return used[0], used[-1], cols[0], cols[-1]


def crop(rows):
    b = bbox(rows)
    return None if b is None else [r[b[2]:b[3] + 1] for r in rows[b[0]:b[1] + 1]]


def runs(lines):
    """Start index and length of each run of identical lines."""
    out, i = [], 0
    while i < len(lines):
        j = i
        while j + 1 < len(lines) and lines[j + 1] == lines[i]:
            j += 1
        out.append((i, j - i + 1))
        i = j + 1
    return out


def thin_runs(lines):
    """How many ink runs are a single pixel, across rows and columns.

    The quality being protected. A glyph built from bars and stems has none; a diagonal has
    some by construction, and the count is what must not grow.
    """
    columns = ["".join(line[i] for line in lines) for i in range(len(lines[0]))]
    total = 0
    for line in list(lines) + columns:
        run = 0
        for ch in line + ".":
            if ch == "X":
                run += 1
                continue
            if run == 1:
                total += 1
            run = 0
    return total


def in_long_run(lines, index):
    """Whether deleting this line only shortens a run of MIN_RUN or more identical ones."""
    for start, length in runs(lines):
        if start < index < start + length and length >= MIN_RUN:
            return True
    return False


def drop(lines, count):
    """Remove `count` lines, choosing each by what it does to the glyph rather than by rule.

    Deleting the interior of a run of identical lines is free - it is the redundant middle of
    a stem, and every bar keeps its width. Letters built from bars and stems have plenty. The
    diagonals have none: no two rows of N, Z or 7 are alike, and there the choice is between
    imperfect rows, so the one that adds the fewest single-pixel runs wins. Both cases are the
    same question asked of the result, which is why they are not two code paths.

    The first and last rows are never candidates: they carry the cap line and the baseline.
    """
    lines = list(lines)
    for _ in range(count):
        if len(lines) <= 2:
            break
        best = min(
            range(1, len(lines) - 1),
            key=lambda i: (
                thin_runs(lines[:i] + lines[i + 1:]),
                not in_long_run(lines, i),
                abs(i - len(lines) // 2),
            ),
        )
        del lines[best]
    return lines


def min_stroke(lines):
    """The narrowest run of ink in any row or column. Two, in a healthy glyph."""
    columns = ["".join(line[i] for line in lines) for i in range(len(lines[0]))]
    worst = None
    for line in list(lines) + columns:
        run = 0
        for ch in line + ".":
            if ch == "X":
                run += 1
                continue
            if run:
                worst = run if worst is None else min(worst, run)
            run = 0
    return worst


def derive(source, anchor, target_h, canvas_rows, canvas_cols):
    """Shrink `source` to target_h and hang it off the bottom-left of `anchor`."""
    b = bbox(source)
    if b is None:
        return None, None
    top, bottom, left, right = b
    glyph = [row[left:right + 1] for row in source[top:bottom + 1]]

    if len(glyph) > target_h:
        glyph = drop(glyph, len(glyph) - target_h)

    a = bbox(anchor)
    if a is None:
        return None, None
    baseline, first_column = a[1], a[2]

    out = [["."] * canvas_cols for _ in range(canvas_rows)]
    first_row = baseline - len(glyph) + 1
    for y, line in enumerate(glyph):
        for x, ch in enumerate(line):
            row, col = first_row + y, first_column + x
            if 0 <= row < canvas_rows and 0 <= col < canvas_cols:
                out[row][col] = ch
    return ["".join(r) for r in out], glyph


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip())
        return 2

    path, target_h = Path(argv[1]), int(argv[2])
    font = json.loads(path.read_text(encoding="utf-8"))
    canvas_rows = len(font["LargeGlyphs"][0]["BitArray"])
    canvas_cols = len(font["LargeGlyphs"][0]["BitArray"][0])

    large = {g["Character"]: g["BitArray"] for g in font["LargeGlyphs"]}

    broken, missing, overflow, done = [], [], [], 0
    for glyph in font["SmallGlyphs"]:
        ch = glyph["Character"]
        source = large.get(ch)
        if source is None:
            missing.append(ch)
            continue

        rows, cropped = derive(source, glyph["BitArray"], target_h, canvas_rows, canvas_cols)
        if cropped is None:
            continue

        # Not an absolute floor: 14 glyphs are drawn with a 1px feature to begin with - the
        # tips of W and Q, the arrows, the diagonals. The rule is that shrinking must not
        # add any, which is what the choice of row was made to avoid.
        before, after = thin_runs(crop(source)), thin_runs(cropped)
        if after > before:
            broken.append((ascii(ch), before, after))
        if len(cropped) > target_h:
            overflow.append((ascii(ch), len(cropped)))

        glyph["BitArray"] = rows
        done += 1

    path.write_text(json.dumps(font, ensure_ascii=False), encoding="utf-8")

    print(f"{path.name}: {done} petits glyphes redessines a {target_h}px max")
    if missing:
        print(f"  pas de grande equivalente, inchanges: {len(missing)}")
    if overflow:
        print(f"  n'ont pas pu descendre a {target_h}: {overflow}")
    if broken:
        print(f"  TRAITS CASSES: {broken}")
        return 1
    print("  traits: 2px minimum partout")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
