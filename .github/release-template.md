## WCtrl DCS-BIOS Bridge {{TAG}}

### Downloads

| Asset | What it is |
| --- | --- |
| `wctrl-dcsbios-bridge-{{TAG}}.zip` | The application. Extract anywhere, run `WCtrlDcsBiosBridge.exe`. |
| `wctrl-export-scripts-{{TAG}}.zip` | The DCS export scripts (Lua). Required for the F-14B(U) CDNU and the C-130J CNI-MU; optional for the A-10C, where it pre-fills wind and elevation on the performance pages. |

### Installing the Lua export script

**Only needed for some aircraft.** Required for the **F-14B(U)** (CDNU) and the **C-130J**
(CNI-MU), and optional for the **A-10C**, where it pre-fills wind and elevation on the
performance pages. Every other aircraft works without it — skip this section.

Follow **[docs/Export-Script-Setup.md](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/docs/Export-Script-Setup.md)**.

In short: extract `wctrl-export-scripts-{{TAG}}.zip` into `%USERPROFILE%\Saved Games\DCS\`,
then add this line to `Saved Games\DCS\Scripts\Export.lua`:

```lua
dofile(lfs.writedir() .. [[Scripts\wctrl-export\wctrl-export.lua]])
```

and tick **Use DCS live data export** under **GENERAL** in the app.

> ⚠️ The script and the app talk a versioned protocol — take both from **this same release**.
> A mismatched script is ignored and the page stays empty.

### Requirements

- Windows x64
- [DCS-BIOS v0.11.6](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/v0.11.6) or later —
  see the [installation section of the README](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/README.md#installation)
  - **F-14B:** a [DCS-BIOS nightly build](https://github.com/DCS-Skunkworks/dcs-bios/releases/tag/latest) dated after 2026-08-22 is recommended

### Documentation

- [README](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/README.md)
- [Lua export script setup](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/docs/Export-Script-Setup.md)
- [LED mapping](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/docs/LED-Mapping.md)
- [F-14B(U) setup](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/docs/F-14BU-Setup.md) ·
  [C-130J setup](https://github.com/landre-cerp/WCtrlDcsBiosBridge/blob/{{TAG}}/docs/C-130J-Setup.md)
- [Wiki](https://github.com/landre-cerp/WCtrlDcsBiosBridge/wiki)

---
