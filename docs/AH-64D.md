# AH-64D Apache

Dual-seat support: both the **Pilot (PLT)** and **Co-Pilot Gunner (CPG)** EUFD and
keyboard unit (KU) pages are mirrored to the CDU. Both pages are always tracked via
DCS-BIOS regardless of which seat is active. No front panel feed.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | PLT or CPG EUFD status/frequency block, keyboard (KU) line, status LEDs |

## CDU pages

Both the PLT and CPG pages share the same layout:

- **Top status block** — the upper EUFD status lines (caution/advisory area).
- **Frequency block** — the radio/preset rows from the lower EUFD.
- **Keyboard line** — the KU scratchpad shown under a `- Keyboard -` separator.

EUFD glyphs (cursor blocks and arrows) are translated to readable characters so each page
looks close to the in-cockpit display.

### LED mapping

| CDU LED | DCS indicator |
|---------|---------------|
| FAIL | Master Caution (pilot) |
| IND | Master Warning (pilot) |

## Single vs Multiple CDUs

### Single CDU Setup
If you only have **1 CDU connected**, the application will:
- Show a **"Select your seat"** prompt once the AH-64D is detected — press the line-select key next to **PILOT** or **CPG**
- Lock the CDU to the chosen seat's page for the entire session

### Multiple CDU Setup
If you have **2 or more CDUs connected**, the application will:
- Show a **"Select your seat"** screen on each CDU once the AH-64D is detected
- Assign the opposite seat to the remaining CDU(s) automatically as soon as one CDU picks a seat
- Keep each CDU fixed to its selected role (PLT or CPG) without automatic switching

No configuration is needed — the behavior is entirely automatic based on detection at startup.

## Brightness / lighting

- Each seat's display, backlight, and LED brightness follow their respective **EUFD brightness** knob
  (`PLT_EUFD_BRT` for the pilot page, `CPG_EUFD_BRT` for the CPG page).
- Only the **active seat's** brightness knob drives the physical CDU device.
- Use the global **"Disable lighting management"** option to leave brightness to the
  aircraft / SimAppPro.

## Front panels

None — the AH-64D does not publish flight-deck data, so FCU/EFIS, PAP3, and AGP32 panels
stay idle.

