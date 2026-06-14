# F/A-18C Hornet

Basic support: two CDU pages — the Up-Front Control (UFC) and the IFEI (engine/fuel
display) — that you flip between with the page keys. No front panel feed.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | UFC scratchpad/options page, IFEI page, Master Caution LED |
| AGP32 | Gear position lights + warning |

## CDU display

The Hornet uses two pages on the CDU:

- **UFC page** (default) — the up-front controller fields.
- **IFEI page** — engine, fuel, and clock readout. Its colour follows the cockpit lighting
  mode (DAY / NITE / NVG).

Switch pages with the configured keys (defaults: **NextPage** -> IFEI, **PrevPage** -> UFC).
The page keys are configurable in the options.

### LED mapping

| CDU LED | DCS indicator |
|---------|---------------|
| FAIL | Master Caution |

## Brightness / lighting

- No dedicated brightness control; the IFEI colour tracks the cockpit DAY/NITE/NVG switch.

## AGP32

The AGP32 gear panel receives live gear-position and warning data from DCS-BIOS:

| AGP32 LED | DCS indicator |
|-----------|---------------|
| Gear 1 (left) green | Left main gear down-and-locked (`FLP_LG_LEFT_GEAR_LT`) |
| Gear 2 (nose) green | Nose gear down-and-locked (`FLP_LG_NOSE_GEAR_LT`) |
| Gear 3 (right) green | Right main gear down-and-locked (`FLP_LG_RIGHT_GEAR_LT`) |
| UNLK triangles (all three) + red arrow | Gear handle warning light (`LANDING_GEAR_HANDLE_LT`) |

## Front panels

FCU/EFIS and PAP3 stay idle — the F/A-18C does not publish speed, heading, or altitude.

## Notes

- Page-switch keys can be remapped via the `NextPageKey` / `PrevPageKey` options.
