
[![Release][release-shield]][release-url]
![License](https://img.shields.io/github/license/landre-cerp/WctrlDcsBiosBridge)
[![Discord][discord-shield]][discord-invite-url]
![Build Status][build-shield]
[![Pre-Release][pre-release-shield]][pre-release-url]

# WctrlDcsBiosBridge

This console application bridges DCS World with some of the WinCtrl hardware, enabling real-time data exchange between the simulator and the physical device.

**Data Flow:** DCS <-> DCS-BIOS <-> This App <-> WinCtrl CDUs (and FCU)

## Quick Start

1. **Install DCS-BIOS** (see detailed instructions below)
2. **Download using the [release](https://github.com/landre-cerp/WCtrlDcsBiosBridge/releases) assets / zip file. and extract** this application to your preferred folder
3. **Connect** your WinCtrl CDU ( before starting bridge )
4. Configure the Dcsbios json folder
5. **Run** the application
6. **Launch DCS** and load your aircraft — the bridge detects it automatically and starts

## Requirements

- DCS World
- DCS-BIOS [v0.11.4](https://github.com/DCS-Skunkworks/dcs-bios/releases) or later (2026-06-20 Nightlybuild or later as it updates the ch47 cdu pages)
- .NET 8.0 runtime

At least one of these devices.
- Winctrl CDU hardware (MCDU / PFP3N / PFP7 / PFP4)
- Winctrl FCU and EFIS ( tested with Left Efis )
- Winctrl PAP3 (or PAP3Mag )
- Winctrl AGP32 Metal


## Supported Aircraft

Each aircraft has its own page describing the supported devices, CDU display, LEDs,
brightness, and any front-panel output. Click an aircraft for details.

[A-10C](docs/A-10C.md) | [AH-64D](docs/AH-64D.md) | [F/A-18C](docs/FA-18C.md) | [CH-47F](docs/CH-47F.md) | [OH-58D](docs/OH-58D.md) | [F-14B](docs/F-14B.md) | [F-15E](docs/F-15E.md) | [F-16C](docs/F-16C.md) | [M-2000C](docs/M-2000C.md) | [UH-1H](docs/UH-1H.md)

Contributions: Smreki F15E , Mustang038 M200C, F16C Poussedebamboo, F18 Iefi pages Martin Javorek

**Front panels** (FCU/EFIS, PAP3, AGP32) render whatever the active aircraft publishes.
The **A-10C** drives the full set (flight data, gear LEDs, and the chrono/UTC/ET clock on the AGP32).
The **F-14B** drives the AGP32 gear lights and clock. The **OH-58D** drives the AGP32 UTC clock.
See each aircraft's page for specifics.

## Installation

### DCS-BIOS Setup

1. **Download** the latest DCS-BIOS release (min v0.11.0 , 0.11.4 recommended as it updates the ch47 cdu pages):
   - Standard: https://github.com/DCS-Skunkworks/dcs-bios/releases

2. **Extract** the DCS-BIOS folder to your DCS saved games Scripts directory:
   ```
   %USERPROFILE%\Saved Games\DCS\Scripts\DCS-BIOS\
   ```

3. **Configure Export.lua** in your Scripts folder:
   ```lua
   dofile(lfs.writedir() .. [[Scripts\DCS-BIOS\BIOS.lua]])
   ```
   
   ⚠️ **Important:** If you already have an Export.lua file, add the line above instead of overwriting it.

### Application Setup

1. **Extract** the application files to your chosen directory
2. **Run** `WctrlDcsBiosBridge.exe`
if no config.json is found, it will create a default one and show you a dialog box to edit it.
3. Select the **File locations** at `DCS-BIOS` JSON folder. It should be located inside the `Scripts/DCS-BIOS/doc/json` folder.
   * Example (as default in your Saved Games): `Saved Games/DCS/Scripts/DCS-BIOS/doc/json`

<img width="441" height="368" alt="image" src="https://github.com/user-attachments/assets/dca3d830-970d-4741-aeb5-7358658f82f0" />

⚠️ **Important:** When updating the application, do not overwrite your existing `config.json` file.

## Usage

### Automatic Aircraft Detection

You no longer pick the aircraft yourself. While the bridge is running, the CDU shows a **"Waiting for DCS... / Aircraft detection"** screen and watches DCS-BIOS for the loaded module:

- When a **supported** aircraft is loaded, the bridge starts automatically.
- An **unsupported** module is shown in red as **"Not supported"**, and the bridge keeps waiting.
- When you **switch or exit** the aircraft, the bridge resets and returns to the waiting screen on its own — **no restart needed**.

The only manual choice left is the **seat** for the CH-47F when two or more CDUs are connected (see below).

### Controls

- **CDU Keys:** Map them in DCS.
- **Seat selection (CH-47F with 2+ CDUs):** when prompted, press the line-select key next to **PILOT** or **COPILOT**. See the [CH-47F documentation](docs/CH-47F.md).

## Troubleshooting

### Common Issues

**"PLT_CDU_LINE1" does not exist (CH-47 Chinook)**
- Wrong dcsbios version installed.
- You need version 0.11.0 or later
  
**"Connection failed" or CDU not responding**
- Ensure your Winctrl CDU is properly connected
- Try unplugging and reconnecting the device
- Restart the application ( application does not detect devices plugged after start )
- Check that no other applications are using the CDU

**"No data appearing on CDU"**
- Start your aircraft in DCS (data appears after aircraft systems are powered)
- Check that DCS-BIOS is working (look for network traffic) - you can use Bort tools from DCSSkunkworks to verify DCS-BIOS is sending data
- Verify Export.lua is configured correctly

**Aircraft not detected / stuck on "Waiting for DCS..."**
- Make sure an aircraft is loaded in the cockpit and DCS-BIOS is exporting (check `Export.lua`)
- Unsupported aircraft are shown in red as "Not supported" — the bridge keeps waiting for a supported one
- You do **not** need to restart when switching aircraft; the bridge detects the change automatically

**Start bridge is greyed**
- You probably launched the app before plugging your devices.
- Exit application, plug all the cdus you plan to use and launch the app again 

### Brightness Issues

- **Mismatched brightness:** Use the aircraft's brightness controls first, then adjust MCDU
- **A10C:** Check the [specific documentation](docs/A-10C.md)
- **CH-47F:** Check the [specific documentation](docs/CH-47F.md)
- In case of flickering with SimAppPro running, check the

<img width="50%" alt="image" src="https://github.com/user-attachments/assets/1cc6f86f-8fc8-457e-a9fb-11191fcd966d" />

### Logs

All application activity is logged to `log.txt` in the same folder as the executable. Check this file for detailed error information.

Report issues [here](https://github.com/landre-cerp/WctrlDcsBiosBridge/issues), or reach out on Discord [![Discord][discord-shield]][discord-invite-url].

## Known Limitations

- **Cursor behavior:** May appear erratic during waypoint entry (reflects DCS-BIOS data)
- **CH-47F support:** Requires DCS-BIOS 0.11.0 or later
- **Brightness sync:** May not perfectly match aircraft state

## Development

This project is written in C# and targets .NET 8.0. It uses:
- **DCS-BIOS** for DCS communication
- **ww-devices-dotnet** for Winctrl hardware interface
- **NLog** for logging

## Contributing
see `docs/CONTRIBUTING.md` for contribution guidelines. [link](docs/CONTRIBUTING.md)

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
