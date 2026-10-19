# F-14B(U) — in a hurry

Nothing to install. The F-14B(U) is the F-14B plus the CDNU, and all of it comes off DCS-BIOS.
The export script is **no longer needed** for this aircraft.

> **DCS-BIOS [v0.11.8](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/v0.11.8) or later**, for the CDNU rows (`RIO_CDNU_LINE1..8`).

Load the F-14B(U) in DCS and the CDNU appears on the CDU.

## What the F-14B(U) shows

Everything the plain F-14B shows, plus the CDNU:

| Page | Key | Notes |
| --- | --- | --- |
| CDNU | **DATA** — the page the aircraft opens on | 8 rows, mixed case, as the RIO sees them |
| RIO CAP | **PREV PAGE** | |
| Radio / IFF | **NEXT PAGE** | |

The gear lights and the clock reach the front panels too, same as on the F-14B, and LEDs bind
like any other aircraft's — see [LED mapping](LED-Mapping.md).

All three keys are yours to change, in the **F-14B / F-14B(U)** section of the options.

## If the CDNU page is empty

It reads `CDNU NOT IN DCS-BIOS` when the installed module does not declare the rows: update
DCS-BIOS to v0.11.8 or later. The rest of the aircraft keeps working meanwhile — the page
is disabled, not the listener, and the log says the same thing in one line.

`WAITING FOR CDNU DATA` means the rows are declared but nothing has arrived yet. Check the CDNU
is powered in the cockpit.

## Symbols

The CDNU draws its arrows and markers on control codes, which DCS-BIOS swaps for printable
stand-ins (`«` `»` `{` `}` `®` `©`) before exporting the row. The bridge turns each one back into
a glyph the CDU font carries.

All five arrows are drawn by `tools/f14bu-font/build-arrows.py`, which reads their placement
out of the font and gives them one head shape, rather than inheriting the A-10C's — whose heads
are wide enough that a left arrow and a right one are hard to tell apart at this size.

The panel is not free about where a bitmap goes: it reaches the device only under one of the 110
characters its glyph table names, and the horizontal double-headed arrow's own U+2194 is not among
them. That one lives in a spare slot instead.
