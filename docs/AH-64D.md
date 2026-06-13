# AH-64D Apache

Basic support: the pilot Enhanced Up-Front Display (EUFD) and keyboard unit are mirrored to
the CDU. No front panel feed.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | Pilot EUFD status/frequency block, keyboard (KU) line, status LEDs |

## CDU display

The pilot EUFD is split across the screen:

- **Top status block** — the upper EUFD status lines (caution/advisory area).
- **Frequency block** — the radio/preset rows from the lower EUFD.
- **Keyboard line** — the KU scratchpad is shown under a `- Keyboard -` separator.

EUFD glyphs (cursor blocks and arrows) are translated to readable characters so the page
looks close to the in-cockpit display.

### LED mapping

| CDU LED | DCS indicator |
|---------|---------------|
| FAIL | Master Caution (pilot) |
| IND | Master Warning (pilot) |

## Brightness / lighting

- Display, backlight, and LED brightness all follow the **EUFD brightness** knob.
- Use the global **"Disable lighting management"** option to leave brightness to the
  aircraft / SimAppPro.

## Front panels

None — the AH-64D does not publish flight-deck data, so FCU/EFIS, PAP3, and AGP32 panels
stay idle.

## Notes

- Only the **pilot** (PLT) EUFD/KU is read.
