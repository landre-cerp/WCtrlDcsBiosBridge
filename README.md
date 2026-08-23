
[![Release][release-shield]][release-url]
![License](https://img.shields.io/github/license/landre-cerp/WctrlDcsBiosBridge)
[![Discord][discord-shield]][discord-invite-url]
![Build Status][build-shield]
[![Pre-Release][pre-release-shield]][pre-release-url]

# WctrlDcsBiosBridge

This desktop application bridges DCS World with some of the WinCtrl hardware, enabling real-time data exchange between the simulator and the physical device.

**Data Flow:** DCS <-> DCS-BIOS <-> This App <-> WinCtrl Devices

## Quick Start

1. **Install DCS-BIOS** (see detailed instructions below)
2. **Download using the [release](https://github.com/landre-cerp/WCtrlDcsBiosBridge/releases) assets / zip file. and extract** this application to your preferred folder
3. **Connect** your WinCtrl CDU ( before starting bridge )
4. Configure the DCS-BIOS json folder (auto-detected in most cases)
5. **Run** the application
6. **Launch DCS** and load your aircraft — the bridge detects it automatically and starts

## Requirements

- DCS World
- DCS-BIOS [v0.11.6](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/v0.11.6) or later
  - **F-14B:** a [nightly build](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/latest) dated after 2026-08-22 is recommended — the latest F-14B
    changes landed after v0.11.6 was cut.
- .NET 8.0 runtime

At least one of these devices.
- Winctrl CDU hardware (MCDU / PFP3N / PFP7 / PFP4)
- Winctrl FCU and EFIS ( tested with Left Efis )
- Winctrl PAP3 (or PAP3Mag )
- Winctrl AGP32 Metal


## Supported Aircraft

Each aircraft has its own page describing the supported devices, CDU display, LEDs,
brightness, and any front-panel output. Click an aircraft for details.

[A-10C](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/A-10C) | [AH-64D](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/AH-64D) | [F/A-18C](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/FA-18C) | [CH-47F](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/CH-47F) | [OH-58D](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/OH-58D) | [F-14B](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/F-14B) | [F-15E](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/F-15E) | [F-16C](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/F-16C) | [M-2000C](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/M-2000C) | [UH-1H](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/UH-1H)

Two more are driven by the live data export rather than DCS-BIOS, which carries no module for
them, and so show one display each: the **F-14B(U)** CDNU ([setup](docs/F-14BU-Setup.md)) and
the **C-130J** CNI-MU ([setup](docs/C-130J-Setup.md)).

Contributions: Smreki F15E , Mustang038 M200C, F16C Poussedebamboo, F18 Iefi pages Martin Javorek

**Front panels** (FCU/EFIS, PAP3, AGP32) render whatever the active aircraft publishes.
The **A-10C** drives the full set (flight data, gear LEDs, and the chrono/UTC/ET clock on the AGP32).
The **F-14B** drives the AGP32 gear lights and clock. The **OH-58D** drives the AGP32 UTC clock.
See each aircraft's page for specifics.

**LEDs** are yours to assign: on the **LEDs** tab, bind any DCS-BIOS indicator of the aircraft
you fly to any LED on your panels — the FCU/EFIS and PAP3 legends (AP1, CMD A, …) light up only
this way. See [LED mapping](docs/LED-Mapping.md).

## Installation

### DCS-BIOS Setup

1. **Download** DCS-BIOS [v0.11.6](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/v0.11.6) or later:
   - Standard: https://github.com/DCS-Skunkworks/dcs-bios/releases
   - **Flying the F-14B?** Take a [nightly build](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/latest) dated after 2026-08-22 instead.

2. **Extract** the DCS-BIOS folder to your DCS saved games Scripts directory:
   ```
   %USERPROFILE%\Saved Games\DCS\Scripts\DCS-BIOS\
   ```
   > If you've relocated your Saved Games folder (right-click it > Properties > Location), extract there instead — the app finds it automatically either way.

3. **Configure Export.lua** in your Scripts folder:
   ```lua
   dofile(lfs.writedir() .. [[Scripts\DCS-BIOS\BIOS.lua]])
   ```
   
   ⚠️ **Important:** If you already have an Export.lua file, add the line above instead of overwriting it.

### DCS export script (optional)

An extra Lua script, shipped as the `wctrl-export-scripts-<version>.zip` asset of each
release. It is **required** for the F-14B(U) CDNU and the C-130J CNI-MU, which DCS-BIOS does
not carry at all, and **optional** for the A-10C, where it pre-fills live wind and field
elevation on the takeoff performance page. See
[Lua export script setup](docs/Export-Script-Setup.md).

### Application Setup

1. **Extract** the application files to your chosen directory
2. **Run** `WctrlDcsBiosBridge.exe`
On first launch, the app looks for DCS-BIOS under your real Saved Games folder (even if you've relocated it) and fills in the JSON path automatically — no dialog box if it finds it.
3. If it can't find it automatically, a config dialog box opens so you can select the **DCS-BIOS JSON folder** manually. It should be located inside the `Scripts/DCS-BIOS/doc/json` folder.
   * Example: `Saved Games/DCS/Scripts/DCS-BIOS/doc/json`
  
<img width="437" height="289" alt="image" src="https://github.com/user-attachments/assets/af96f5c0-2657-4589-9cc4-4537904048ae" />

## Usage

### Automatic Aircraft Detection

You no longer pick the aircraft yourself. While the bridge is running, the CDU shows a **"Waiting for DCS... / Aircraft detection"** screen and watches DCS-BIOS for the loaded module:

- When a **supported** aircraft is loaded, the bridge starts automatically.
- An **unsupported** module is shown in red as **"Not supported"**, and the bridge keeps waiting.
- When you **switch or exit** the aircraft, the bridge resets and returns to the waiting screen on its own — **no restart needed**.

The only manual choice left is the **seat** for a dual-seat aircraft (AH-64D, CH-47F, C-130J) when two or more CDUs are connected (see below). With a single CDU there is nothing to choose: the AH-64D and CH-47F follow you from one seat to the other, and the C-130J shows the pilot's CNI-MU.

### Controls

- **CDU Keys:** Map them in DCS.
- **Seat selection (dual-seat aircraft with 2+ CDUs):** when prompted, press the line-select key next to **PILOT** or **COPILOT**; the other CDU takes the seat you did not pick. See the [CH-47F documentation](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/CH-47F).

## Troubleshooting

### Brightness Issues

- **Mismatched brightness:** Use the aircraft's brightness controls first, then adjust MCDU
- **A10C:** Check the [specific documentation](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/A-10C)
- **CH-47F:** Check the [specific documentation](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/CH-47F)
- In case of flickering with SimAppPro running, check the

<img width="50%" alt="image" src="https://github.com/user-attachments/assets/1cc6f86f-8fc8-457e-a9fb-11191fcd966d" />

### Logs

All application activity is logged to `log.txt` in the same folder as the executable. Check this file for detailed error information.

Report issues [here](https://github.com/landre-cerp/WctrlDcsBiosBridge/issues), or reach out on Discord [![Discord][discord-shield]][discord-invite-url].

## Known Limitations

- **Cursor behavior:** May appear erratic during waypoint entry (reflects DCS-BIOS data)
- **CH-47F support:** Requires DCS-BIOS 0.11.6 or later
- **Brightness sync:** May not perfectly match aircraft state

## Development

This project is written in C# and targets .NET 8.0. It uses:
- **DCS-BIOS** for DCS communication
- **ww-devices-dotnet** for Winctrl hardware interface
- **NLog** for logging

## Contributing
see the [wiki Contributing page](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki/Contributing) for contribution guidelines.

## License

See `LICENSE.txt` and `thirdparty-licences.txt` for licensing information.

## Support

For issues and questions, please check the logs first and review the troubleshooting section above.

and if you want, no need, you can [Buy Me a Coffee](https://www.buymeacoffee.com/cerppo)

[release-url]: https://github.com/landre-cerp/WctrlDcsBiosBridge/releases
[release-shield]:  https://img.shields.io/github/release/landre-cerp/WctrlDcsBiosBridge.svg
[discord-shield]: https://img.shields.io/discord/231115945047883778
[discord-invite-url]: https://discord.gg/Td2cGvMhVC
[dcs-forum-discussion]: https://forum.dcs.world/topic/368056-winwing-mcdu-can-it-be-used-in-dcs-for-other-aircraft/page/4/
[build-shield]: https://img.shields.io/github/actions/workflow/status/landre-cerp/WctrlDcsBiosBridge/build-on-tag.yml
[pre-release-shield]: https://img.shields.io/github/v/release/landre-cerp/WctrlDcsBiosBridge?include_prereleases&sort=semver
[pre-release-url]: https://github.com/landre-cerp/WctrlDcsBiosBridge/releases
