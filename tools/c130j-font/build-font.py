#!/usr/bin/env python3
"""Derive the C-130J CDU font from the A-10C one.

A CDU font's bitmaps need not look like the characters they are filed under. The A-10C's are
drawn for the A-10C: its U+2610 is an open-ended bracket pair, and it has no lowercase at all
because the aircraft never asks for any. The CNI wants both, so rather than edit the A-10C font
and change what that aircraft draws, this writes a second one beside it.

Two transforms plus one metadata change — the character set is identical afterwards apart from
the letters added:

  1. U+2610, the empty entry field. Redrawn as a closed box in both sizes, on the footprint the
     rest of the font uses: rows 8-28 columns 2-18 large, rows 15-28 columns 2-11 small, with
     the two-pixel stroke everything else is drawn with.

  2. a-z. Given the bitmaps of their capitals. The CNI draws lowercase in one place in the whole
     module — the "c" of a temperature — and without a glyph the compositor either drops it or
     falls back to a small capital, which comes out narrower than the text around it. Real
     lowercase letterforms would be better and are a drawing job; this at least keeps a word the
     same size all the way through.

  3. '%', large. The A-10C's is a question mark, and no A-10C page asks for a percent so nobody
     ever saw it; the CNI asks on its first page, where TOLD INIT prints a runway slope. See
     percent.py, which also draws the small one from resize-small.py.

  4. GlyphFullWidth, 23 -> 22. This is the step the panel advances by between characters, and
     with 25 columns of it the CNI's 575px of text left 4px of margin either side of the 584px
     aperture. At 22 the margin is 17px, what the 24-column aircraft have always had. Nothing
     is clipped: the ink of the widest glyphs, the boxes, ends at column 18.

    python build-font.py ../../Resources/a10c-font-21x31.json ../../Resources/c130j-font-21x31.json
"""

import json
import string
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import percent

BOX = "☐"

# Where the box sits, per size: first row, last row, first column, last column.
FOOTPRINT = {"LargeGlyphs": (8, 28, 2, 18), "SmallGlyphs": (15, 28, 2, 11)}
STROKE = 2

ADVANCE = 22


def closed_box(width, height, top, bottom, left, right):
    rows = []
    for y in range(height):
        if y < top or y > bottom:
            rows.append("." * width)
            continue

        row = ["."] * width
        if y < top + STROKE or y > bottom - STROKE:
            for x in range(left, right + 1):
                row[x] = "X"
        else:
            for t in range(STROKE):
                row[left + t] = "X"
                row[right - t] = "X"
        rows.append("".join(row))
    return rows


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    font = json.loads(src.read_text(encoding="utf-8"))

    width, height = font["GlyphWidth"], font["GlyphHeight"]
    font["Name"] = "C130J"
    font["GlyphFullWidth"] = ADVANCE

    for size, (top, bottom, left, right) in FOOTPRINT.items():
        glyphs = font[size]

        boxes = [g for g in glyphs if g["Character"] == BOX]
        if not boxes:
            raise SystemExit(f"{size}: {BOX!r} absent from {src.name}")
        for glyph in boxes:
            glyph["BitArray"] = closed_box(width, height, top, bottom, left, right)

        # Large only. The small '/' is still the source font's 14px one at this point;
        # resize-small.py redraws it at the cap height the rest of the small set uses, and
        # rebuilds '%' against it there with the same helper.
        if size == "LargeGlyphs":
            by_character = {g["Character"]: g for g in glyphs}
            for character in ("/", "%"):
                if character not in by_character:
                    raise SystemExit(f"{size}: {character!r} absent from {src.name}")
            by_character["%"]["BitArray"] = percent.draw(
                by_character["/"]["BitArray"], size
            )

        # And the two the source font has no bitmap for at all, into both sets: a character the
        # small set has never heard of is one resize-small.py will not redraw, and it derives
        # what it needs from the large bitmap put here.
        present = {g["Character"]: g for g in glyphs}
        for character, bitmap in percent.missing(width, height).items():
            if character in present:
                present[character]["BitArray"] = bitmap
            else:
                glyphs.append({"Character": character, "BitArray": bitmap})

        capitals = {g["Character"]: g for g in glyphs if g["Character"] in string.ascii_uppercase}
        present = {g["Character"] for g in glyphs}
        for upper, glyph in sorted(capitals.items()):
            lower = upper.lower()
            if lower not in present:
                glyphs.append({"Character": lower, "BitArray": list(glyph["BitArray"])})

    dst.write_text(json.dumps(font, ensure_ascii=False, indent=2), encoding="utf-8")

    for size in FOOTPRINT:
        print(f"{size}: {len(font[size])} glyphes")


if __name__ == "__main__":
    main()
