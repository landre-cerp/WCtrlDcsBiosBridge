# Contributing Guidelines

Thank you for your interest in contributing to this C# project! 🎉  
We welcome all contributions, whether it’s bug reports, feature requests, code improvements, or documentation updates.

---

## 🚀 How to Contribute

### Prerequisites
- Ensure you have Git installed on your machine.
- Visual Studio
- .NET 8 SDK
- DCS-BIOS installed in your DCS World Saved Games folder. ( check project README for details)
https://github.com/DCS-Skunkworks/dcs-bios (v0.11.0 or later for CH-47F)

### Recommended Tools
Bort https://github.com/DCS-Skunkworks/Bort or any other Dcsbios Reference tool to help you find the right addresses and values.
This is where you get "CDU_BRT" you use as a parameter to `RegisterUInt` / `RegisterStr`:
```csharp
RegisterUInt("CDU_BRT", v => { /* handle v */ });
RegisterStr("UFC_LINE1", s => { /* handle s */ });
```

### 1. Fork & Clone
- Fork this repository.
- Clone your fork locally
- ```bash
  git clone --recurse-submodules <your-forked-repo-url>
  ```

### 2. Create a Branch
**If you use Visual Studio, select the McduDcsBiosBridge repository**
- Create a new branch for your feature or bug fix:
  ```bash
  git checkout -b my-feature-branch
  ```
  Replace `my-feature-branch` with a descriptive name.
- I'm not strict on branch names. Here are some examples:
  - `fix-issue-123`
  - `add-new-feature`
  - `improve-documentation`

or commitize style like:
  - `feat(aircraft)/add-new-feature`
  - `fix/issue-123`
  - `docs/update-readme`

### 3. Make Changes
- Make your changes in the appropriate files.
- Follow the existing coding style and conventions.
- For C# code, target `.NET 8`
- **Naming convention for private fields:** prefix private `DCSBIOSOutput?` fields with `_` (e.g. `_CDU_BRT`, `_IAS`). This distinguishes them from local variables and matches the convention used across all aircraft listeners.

### Add a new Aircraft
Aircraft are now driven by a single registry and detected automatically from the DCS-BIOS `MetadataStart/_ACFT_NAME` value — there is no CDU menu to edit anymore.

#### 1. Find the module id and name
Look up the aircraft's module number in `dcs-bios_modules.txt` (for example: F-14 = 16) and its `_ACFT_NAME` value (use Bort, or run the program and start the aircraft in DCS — the wait screen shows the name as "unsupported").

#### 2. Create a listener
Add a class in `Aircrafts/` deriving from `AircraftListener`. You implement two methods: wire the CDU and wire the frontpanels. Use `RegisterUInt` / `RegisterStr` to resolve and register in one call — no field declarations needed for single-use outputs. (See the existing `Aircrafts/*_Listener.cs` for real examples.)

```csharp
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class F14_Listener : AircraftListener
{
    // Only declare a field when the same output is used in MORE than one method.
    // Example: a brightness knob registered in both RegisterCduControls and
    // RegisterFrontpanelControls needs its MaxValue in both handlers.
    private DCSBIOSOutput? _CONSOLE_BRT;

    // The registry factory passes UserOptions; forward it with the matching
    // descriptor to the base class.
    public F14_Listener(UserOptions options)
        : base(AircraftRegistry.F14, options)
    {
    }

    // Only override InitializeDcsBiosOutputs when you need array loops or
    // multi-use fields. Leave it out entirely for simple listeners.
    protected override void InitializeDcsBiosOutputs()
    {
        _CONSOLE_BRT = ResolveUInt("CONSOLE_BRT");
    }

    // Wire CDU outputs. Runs only when a CDU is connected.
    protected override void RegisterCduControls()
    {
        // Single-use string output → CDU line, resolved and registered in one call.
        RegisterStr("PLT_CAP_DISPLAY", s =>
            GetCompositor(DEFAULT_PAGE).Line(0).Green().WriteLine(s));

        // Single-use integer output → CDU status LED.
        RegisterUInt("MASTER_CAUTION", v => SetCduLeds(fail: v == 1));

        // Multi-use field: _CONSOLE_BRT was resolved in InitializeDcsBiosOutputs.
        if (!options.DisableLightingManagement)
        {
            Register(_CONSOLE_BRT, v =>
            {
                int percent = (int)(v * 100 / _CONSOLE_BRT!.MaxValue);
                SetBacklightBrightnessPercent(percent);
                SetDisplayBrightnessPercent(percent);
                SetLedBrightnessPercent(percent);
            });
        }
    }

    // Wire frontpanel outputs (FCU/EFIS, PAP3, AGP32...). Always runs, even
    // with no CDU, because a frontpanel-only setup still needs the data.
    // Write semantic values into FlightDeck; renderers show what they can.
    protected override void RegisterFrontpanelControls()
    {
        // Leave empty if the aircraft has no frontpanel data, or populate
        // FlightDeck like the A-10C does, e.g.:
        // RegisterUInt("IAS_US_INT", v => FlightDeck.Speed = (int)v);

        // For outputs that need MaxValue inside the handler, use the (ctrl, v) overload:
        // RegisterUInt("CONSOLE_BRT", (ctrl, v) =>
        //     FlightDeck.ConsoleBrightness = (byte)(v * 255 / ctrl.MaxValue));
    }
}
```

#### 3. Register one descriptor
Add an `AircraftDescriptor` in `Aircrafts/AircraftRegistry.cs` and include it in the `All` list (registry order = menu order). Provide the module id, display name, JSON and font files, the `HasSeatSelection` flag, the DCS-BIOS name(s) used for automatic detection, and the listener factory.

```csharp
public static readonly AircraftDescriptor F14 = new(
    16,                                   // module id (dcs-bios_modules.txt)
    "F-14",                               // display name
    "F-14.json",                          // DCS-BIOS json file for this module
    "resources/a10c-font-21x31.json",     // a font from resources/
    false,                                // HasSeatSelection (true only for CH-47F today)
    new[] { "F-14B", "F-14A-135-GR" },    // _ACFT_NAME value(s) for auto-detection
    c => new F14_Listener(c.Options));    // listener factory

// ...then add it to the All list:
public static readonly IReadOnlyList<AircraftDescriptor> All = new[]
{
    A10C, AH64D, FA18C, CH47, F15E, M2000C, F16C, OH58D, F14,
};
```

> **Dual-seat aircraft:** set `HasSeatSelection: true` and give the listener an `isPilot` parameter, like the CH-47F: `c => new CH47F_Listener(c.Options, c.IsPilot, c.Ch47SwitchWithSeat)`.

> **Font:** start by reusing an existing resource — `resources/a10c-font-21x31.json` or `resources/ah64d-font-21x31.json`. A font file maps a **fixed** character set to bitmap glyphs; if every character you render already looks correct, there is **no point duplicating** a font just to give it a new name. Create a dedicated font only when you need to **design a special bitmap** for a character code (e.g. a custom symbol drawn when a given char code is sent). You change the glyph's bitmap (the drawing) for an existing char code — you cannot add new characters, the charset in the file is fixed.

#### Helpers available from `AircraftListener`

| Helper | Use |
|--------|-----|
| `RegisterUInt(id, v => ...)` | Resolve + register an integer output in one call. |
| `RegisterUInt(id, (ctrl, v) => ...)` | Same, but passes the resolved `DCSBIOSOutput` as `ctrl` so you can read `ctrl.MaxValue` inside the handler. |
| `RegisterStr(id, s => ...)` | Resolve + register a string output in one call. |
| `ResolveUInt(id)` | Resolve an integer output for later use (multi-use fields or array loops). |
| `ResolveStr(id)` | Resolve a string output for later use. |
| `RegisterUInt(output, v => ...)` | Register a pre-resolved integer output. |
| `RegisterStr(output, s => ...)` | Register a pre-resolved string output. |
| `RegisterRaw(address, v => ...)` | Low-level handler for raw bitfield registers when named DCS-BIOS outputs are unavailable or have incorrect mask/shift definitions (M-2000C uses this). `v` is the raw unmasked 16-bit register value; apply bitmasks manually. Also handles the address whitelist registration required by the protocol parser — no extra setup needed. |
| `GetCompositor(DEFAULT_PAGE).Line(n)...` | Write a CDU line (`.Green()`, `.White()`, `.WriteLine(...)`, ...). |
| `SetCduLeds(fail:, fm1:, fm2:, fm:, ind:, rdy:)` | Set CDU status LEDs. |
| `SetDisplayBrightnessPercent` / `SetBacklightBrightnessPercent` / `SetLedBrightnessPercent` | CDU brightness (0-100). |
| `FlightDeck` | Semantic front-panel state — see table below. |
| `HasCdu`, `CduDevice` | Whether a CDU is connected / the underlying device. |

#### FlightDeck properties

Populate these in `RegisterFrontpanelControls()`. Renderers read whatever properties their device can display; leave anything your aircraft does not provide as `null`.

| Property | Type | Meaning |
|----------|------|---------|
| `Speed`, `Heading`, `Altitude`, `VerticalSpeed` | `int?` | Indicated airspeed (kt), heading (°), altitude (ft), vertical speed (ft/min). |
| `BaroPressure` | `int?` | Barometric pressure in inHg × 100 (e.g. 2992 = 29.92 inHg). |
| `GearLeftDown`, `GearNoseDown`, `GearRightDown` | `bool?` | Gear down-and-locked indicators. |
| `GearWarning` | `bool?` | Gear handle red warning light. |
| `ClockUtcTime` | `string?` | UTC time as `"HHMMSS"` (e.g. `"123456"` = 12:34:56). |
| `ClockChrono` | `string?` | Chronograph as `"MMSS"` (e.g. `"0145"` = 1 min 45 s). |
| `ClockElapsedTime` | `string?` | Elapsed time as `"HHMM"` (e.g. `"0012"` = 12 min). |
| `ConsoleBrightness` | `byte?` | Cockpit console backlight, 0–255. |
| `SegmentBrightnessPercent` | `int` | 7-segment display brightness, 0–100 (default 100). |

### 4. Test Your Changes
- There's no automated test suite (as most of the tests are using the physical device), please ensure to:
- Review your code for any errors or typos.

### 5. Commit & Push
- Commit your changes with a clear message:

  ```bash
  git commit -m "Description of my changes"
  ```
- Push your changes to your forked repository:
  ```bash
  git push origin my-feature-branch
  ```

### 6. Create a Pull Request
- Go to the original repository where you want to merge your changes.
- Click on "New Pull Request".
- Select your branch with the changes and create the pull request.
- Provide a clear description of the changes and why they are needed.

---

## 📝 Guidelines

- Please ensure your code adheres to the existing code style and conventions. (any improvements are welcome)
- Write clear, concise commit messages.
- Keep your changes focused on one issue or feature.

---

## ❓ FAQ

**Q: Keeps complaining that it can't find Aircraft 50 (CH47F)**
A: Add the CH-47F to the supported aircraft list in dcs-bios_modules.txt after the last entry (49)
It's because the CH-47F is not yet handled in the DCSFPCommon library.
```
OH-58D|49|OH-58D Kiowa Warrior
CH-47F|50|CH-47F Chinook
```

**Q: What if I found a bug?**  
A: Please check if the bug is already reported. If not, feel free to open a new issue with steps to reproduce the bug.

**Q: Can I contribute to the documentation?**  
A: Absolutely! We welcome documentation improvements. You can follow the same process: fork, clone, branch, and contribute.

**Q: How do I know if my changes are acceptable?**  
A: Follow the coding standards of the project, test your changes, and ensure your changes are meaningful and well-explained.

---

Thank you for considering contributing to our project! Your help is greatly appreciated. If you have any questions, feel free to ask in the discussions or issues section.
