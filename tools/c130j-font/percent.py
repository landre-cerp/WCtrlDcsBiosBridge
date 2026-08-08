#!/usr/bin/env python3
"""Draws the '%' glyph, for the two scripts that each own one size of it.

The A-10C's large '%' is a question mark: the bitmap filed under that character was never drawn
for it, and no A-10C page asks for a percent, so nobody ever saw it. The CNI asks on its first
page — TOLD INIT prints a runway slope — and it came out as a question mark on the panel.

Drawn from the font's own '/' rather than from scratch, so the slope, the stroke and the weight
are the font's and not a second hand's. The rings are the only new ink.

The two sizes are drawn at different points in the pipeline because each wants the slash that is
current at that point: build-font.py does the large one, and resize-small.py does the small one
after it has redrawn '/' at the taller cap height the rest of the small set uses. Deriving the
small percent from the large one instead does not work — a diagonal runs through every row of a
ring, so no two rows are alike, and the derivation's rule of only deleting from inside a run of
three identical rows has to take a cap instead, leaving the rings open at one end.
"""

# Ring height, then where the top-left and bottom-right rings sit. Six wide leaves a clear
# column between each ring and the slash; anything wider merges with it and the glyph reads as
# a blot. Height follows the cap box, 21 rows large against 18 small.
GEOMETRY = {
    "LargeGlyphs": (7, ((8, 2), (22, 9))),
    "SmallGlyphs": (6, ((11, 2), (23, 9))),
}


def draw(slash, size):
    """The given '/' bitmap with a ring above and below it."""
    ring_height, origins = GEOMETRY[size]
    ring = [".####."] + ["##..##"] * (ring_height - 2) + [".####."]

    rows = [list(row) for row in slash]
    for origin_row, origin_col in origins:
        for dy, line in enumerate(ring):
            for dx, pixel in enumerate(line):
                if pixel == "#":
                    rows[origin_row + dy][origin_col + dx] = "X"
    return ["".join(row) for row in rows]


# Glyphs the source font simply does not have. The A-10C never asks for them, so the character
# was left out rather than drawn wrong: unlike '%', there is no bitmap under it at all and the
# panel draws nothing. The CNI asks for both — "<WT & BAL" on MSN CMPTR INDEX and " CHUTE/#" on
# CARP INIT 2 — and the device does have a slot for each, so all that is missing is the ink.
#
# Drawn on the large cap box, rows 8-28 and columns 2-14, with the two-pixel stroke the rest of
# the font uses. The small set is derived from these by resize-small.py in the ordinary way:
# neither is a ring, so the row it deletes comes out of a stem and costs nothing.
MISSING = {
    "&": (
        "...#####.....",
        "..##...##....",
        ".##.....##...",
        ".##.....##...",
        ".##.....##...",
        "..##...##....",
        "...#####.....",
        "...####......",
        "..###.##.....",
        ".###...##....",
        "###.....##..#",
        "##.......##.#",
        "##........###",
        "##.........##",
        "##........###",
        "##.......####",
        "###.....##.##",
        ".###...##..##",
        "..#######...#",
        "...#####....#",
    ),
    "#": (
        "....##...##..",
        "....##...##..",
        "....##...##..",
        "....##...##..",
        ".############",
        ".############",
        "...##...##...",
        "...##...##...",
        "...##...##...",
        "...##...##...",
        "..##...##....",
        "..##...##....",
        "############.",
        "############.",
        "..##...##....",
        "..##...##....",
        "..##...##....",
        "..##...##....",
    ),
}

# Top-left corner of the large cap box.
MISSING_ORIGIN = (8, 2)


def missing(width, height):
    """The glyphs the source font has no bitmap for, as full-canvas bitmaps."""
    out = {}
    for character, art in MISSING.items():
        rows = [["."] * width for _ in range(height)]
        top, left = MISSING_ORIGIN
        for dy, line in enumerate(art):
            for dx, pixel in enumerate(line):
                if pixel == "#":
                    rows[top + dy][left + dx] = "X"
        out[character] = ["".join(row) for row in rows]
    return out
