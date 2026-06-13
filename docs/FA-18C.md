# F/A-18C Hornet

Basic support: two CDU pages — the Up-Front Control (UFC) and the IFEI (engine/fuel
display) — that you flip between with the page keys. No front panel feed.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | UFC scratchpad/options page, IFEI page, Master Caution LED |

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

## Front panels

None — the F/A-18C does not publish flight-deck data, so FCU/EFIS, PAP3, and AGP32 panels
stay idle.

## Notes

- Page-switch keys can be remapped via the `NextPageKey` / `PrevPageKey` options.
