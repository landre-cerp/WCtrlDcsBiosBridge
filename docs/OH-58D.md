# OH-58D Kiowa Warrior

Basic support: the engine/MPD readouts, CMWS lines, and the RFI radio list are shown on the
CDU. The cockpit clock also feeds the AGP32 UTC display.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | TGT/TRQ, NG, MPD selectable readouts, CMWS, RFI radio list |
| AGP32 | UTC clock (from the cockpit clock) |

## CDU display

- **TGT / TRQ** — turbine gas temperature and torque on the top line.
- **NG** — gas-generator speed.
- **MPD selector** — the rotary-selected pair of parameters (battery/start volts, generator
  loads, AC/rectifier volts, fuel/torque, NR/NP) with labels that follow the selector.
- **CMWS** — the two countermeasures lines.
- **RFI** — the five-line radio frequency list, with pilot/co-pilot selection arrows and a
  cipher marker.

## Brightness / lighting

- Display, backlight, and LED brightness all follow the **RFI brightness** control.
- Use the global **"Disable lighting management"** option to opt out.

## Front panels

- **AGP32 clock:** the cockpit clock drives the panel's **UTC** time display.
- No gear LEDs or other flight data are published, so the rest of the AGP32 (and FCU/EFIS,
  PAP3) stay idle.

## Notes

- The clock feed runs even on a front-panel-only setup (no CDU connected), so the AGP32 UTC
  still updates.
