# UH-1H Huey

Radio panel support: VHF AM, UHF, VHF FM, and VHF NAV frequencies are shown on the CDU.
The Master Caution light maps to the CDU FAIL LED.

## Supported devices

| Device | What it shows |
|--------|---------------|
| CDU (MCDU / PFP3N / PFP7 / PFP4) | Radio panel (VHF AM, UHF, VHF FM, VHF NAV) |

## CDU display

| Line | System | DCS-BIOS output(s) |
|------|--------|--------------------|
| 0 | Title | *(static)* |
| 2 | **VHF AM** (AN/ARC-134) | `VHFCOMM_FREQ` (String), `VHFCOMM_PWR` (UInt) |
| 3 | **UHF** (AN/ARC-51) | `UHF_FREQ` (String), `UHF_PRESET`, `UHF_MODE`, `UHF_FUNCTION` (UInt) |
| 4 | **VHF FM** (AN/ARC-131) | `VHFFM_FREQ1`–`4`, `VHFFM_MODE` (UInt) |
| 5 | **VHF NAV** (ARN-82) | `VHFNAV_FREQ` (String), `VHFNAV_PWR` (UInt) |

Each radio shows **OFF** when its power or mode switch is in the off position.

The UHF line shows the preset channel number (`CH:xx`) when the mode dial is on PRESET,
and the function label (T/R, T/R+G, ADF) next to the frequency.

## LEDs

| CDU LED | Cockpit source |
|---------|----------------|
| FAIL | `MASTER_CAUTION_IND` — Master Caution light |

## Brightness / lighting

There is no CDU in the real UH-1H cockpit, so brightness is **not** driven by any
DCS-BIOS control. The display starts at **100%** and is adjusted entirely with the
physical **BRT** / **DIM** keys on the CDU hardware (step: 5%).

Use the global **"Disable lighting management"** option to prevent Master Caution from
toggling the FAIL LED.

## Known DCS-BIOS limitations (disabled features)

| Feature | Reason disabled |
|---------|-----------------|
| **ADF** (AN/ARN-83) | `ADF_FREQ` output reports incorrect values |
| **IFF** (AN/APX-72) | `IFF_MODE1_WHEEL*` and `IFF_MODE3A_WHEEL*` outputs are bugged |

These sections are commented out in `Aircrafts/UH1H/UH1H_Listener.cs` and can be
re-enabled once the upstream DCS-BIOS module is fixed.

## Front panels

No flight data (gear, speed, altitude) is published to `FlightDeckState`, so FCU/EFIS,
PAP3, and AGP32 panels stay idle.
