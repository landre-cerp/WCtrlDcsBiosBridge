#!/usr/bin/env python3
"""Make the 32px variant of a font by padding each glyph with a blank row.

The line select keys sit 64px apart on every panel, so rows want a 32px pitch. The PFP family
has the aperture for it; the MCDU is 6mm shorter and does not, which is why both heights exist
and why AircraftListener picks between them from ICdu.MaxGlyphHeight rather than from the
aircraft. The glyph itself does not move — YOffsetForGlyphHeight returns 0 for 31 and 32 alike
— so the extra pixel goes to the pitch and nothing else.

    python pad-font.py ../../Resources/c130j-font-21x31.json ../../Resources/c130j-font-21x32.json
"""

import json
import sys
from pathlib import Path


def pad(font):
    font["GlyphHeight"] += 1
    for key in ("LargeGlyphs", "SmallGlyphs"):
        for glyph in font.get(key, []):
            rows = glyph["BitArray"]
            rows.append("." * len(rows[0]))
    return font


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip())
        return 2

    source, target = Path(argv[1]), Path(argv[2])
    font = json.loads(source.read_text(encoding="utf-8"))

    height = font["GlyphHeight"]
    if height != 31:
        print(f"{source.name}: expected a 31px font, found {height}px")
        return 1

    target.write_text(json.dumps(pad(font), ensure_ascii=False), encoding="utf-8")
    print(f"{target.name}: {font['GlyphWidth']}x{font['GlyphHeight']}, "
          f"{len(font['LargeGlyphs'])} large, {len(font['SmallGlyphs'])} small")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
