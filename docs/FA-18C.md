# F/A-18C Hornet

Two CDU pages — the Up-Front Control (UFC) and the IFEI (engine/fuel display) — switchable with the page keys. The AGP32 receives gear position, gear warning, flap position, and clock/timer data.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | UFC scratchpad/options page, IFEI page, Master Caution LED |
| AGP32 | Gear position lights + warning, flap lights, UTC clock, ET, chrono |

## CDU display

The Hornet uses two pages on the CDU:

- **UFC page** (default) — the up-front controller fields.
- **IFEI page** — engine, fuel, and clock readout. Its colour follows the cockpit lighting
  mode (DAY / NITE / NVG).

Switch pages with the configured keys (defaults: **NextPage** → IFEI, **PrevPage** → UFC).
The page keys are configurable in the options.

### LED mapping

| CDU LED | DCS indicator |
|---------|---------------|
| FAIL | Master Caution |

## Brightness / lighting

No dedicated brightness control; the IFEI colour tracks the cockpit DAY/NITE/NVG switch.

## AGP32

### Gear and warning lights

| AGP32 LED | DCS indicator |
|-----------|---------------|
| Gear 1 (left) green | Left main gear down-and-locked (`FLP_LG_LEFT_GEAR_LT`) |
| Gear 2 (nose) green | Nose gear down-and-locked (`FLP_LG_NOSE_GEAR_LT`) |
| Gear 3 (right) green | Right main gear down-and-locked (`FLP_LG_RIGHT_GEAR_LT`) |
| UNLK triangles (all three) + red arrow | Gear handle warning light (`LANDING_GEAR_HANDLE_LT`) |

### Flap lights

The F/A-18C has no auto-brake, so the two lower AutoBrk DECEL LEDs are repurposed for flap
position:

| AGP32 LED (A320 label) | F/A-18C meaning | DCS indicator |
|------------------------|-----------------|---------------|
| AutoBrk LO DECEL | Half flaps | `FLP_LG_HALF_FLAPS_LT` |
| AutoBrk MED DECEL | Full flaps | `FLP_LG_FULL_FLAPS_LT` |

### Clock displays

Clock and timer data comes from the IFEI outputs:

| AGP32 display | Content | Source |
|---------------|---------|--------|
| Clock (HH:MM:SS) | UTC time | `IFEI_CLOCK_H/M/S` |
| ET (MM:SS) | Timer minutes and seconds | `IFEI_TIMER_M/S` |
| CHR (HH:MM) | Timer hours and minutes — shown only when timer hours are non-zero | `IFEI_TIMER_H/M` |

## Front panels

FCU/EFIS and PAP3 stay idle — the F/A-18C does not publish speed, heading, or altitude.

## Notes

- Page-switch keys can be remapped via the `NextPageKey` / `PrevPageKey` options.
