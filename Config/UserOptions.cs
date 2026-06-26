using WCtrlDcsBiosBridge.Config;

/// <summary>
/// User-configurable options, persisted to useroptions.json.
/// App-wide flags live here directly; per-aircraft settings live in one small
/// object per aircraft (see below) so a new aircraft adds a single property.
/// </summary>
public class UserOptions
{
    // ── App-wide ─────────────────────────────────────────────────────────────

    /// <summary>Disable lighting management (for SimApp Pro users).</summary>
    public bool DisableLightingManagement { get; set; }

    /// <summary>Automatically start the bridge once conditions are met.</summary>
    public bool AutoStart { get; set; }

    /// <summary>Minimize the main window after the bridge starts.</summary>
    public bool MinimizeOnStart { get; set; }

    /// <summary>Preferred application theme (System, Light, or Dark).</summary>
    public ThemePreference Theme { get; set; } = ThemePreference.System;

    // ── On-close device reset ────────────────────────────────────────────────
    // Only applies when DisableLightingManagement is false.
    // Screen content is always cleared on close.

    /// <summary>Set all device backlights to 0 % on close (else stays at 80 %).</summary>
    public bool CloseResetBacklight { get; set; } = true;

    /// <summary>Turn off all button/panel LEDs (markers) on close.</summary>
    public bool CloseResetMarkers { get; set; } = true;

    // ── Per-aircraft ─────────────────────────────────────────────────────────
    // To add an aircraft's options: add one property here and one options class.

    public A10COptions A10C { get; set; } = new();
    public FA18COptions FA18C { get; set; } = new();
    public F16COptions F16C { get; set; } = new();
    public F14Options F14 { get; set; } = new();
}

// Key properties below must be a valid WwDevicesDotNet.Key enum name
// (e.g. "NextPage", "LineSelectRight1"); listeners fall back to a default if not.

/// <summary>A-10C CDU options.</summary>
public class A10COptions
{
    /// <summary>Align the CDU display to the bottom of the screen.</summary>
    public bool DisplayBottomAligned { get; set; }

    /// <summary>Show the CMS lines on the CDU.</summary>
    public bool DisplayCMS { get; set; }

    /// <summary>Whether the takeoff/landing performance pages are available.</summary>
    public bool EnablePerfPages { get; set; } = true;

    /// <summary>MCDU key that opens the takeoff performance page.</summary>
    public string PerfPageKey { get; set; } = "FuelPred";

    /// <summary>MCDU key to switch to the next page.</summary>
    public string NextPageKey { get; set; } = "NextPage";

    /// <summary>MCDU key to switch to the previous page.</summary>
    public string PrevPageKey { get; set; } = "PrevPage";

    /// <summary>
    /// Forward the live CDU line-select keys, digits and slash to the sim over
    /// DCS-BIOS. Enable after unbinding those keys in DCS. Independent of
    /// <see cref="EnablePerfPages"/>.
    /// </summary>
    public bool ForwardCduKeysToSim { get; set; }
}

/// <summary>F/A-18C IFEI/UFC page options.</summary>
public class FA18COptions
{
    /// <summary>MCDU key that shows the IFEI page.</summary>
    public string ShowIfeiKey { get; set; } = "NextPage";

    /// <summary>MCDU key that shows the UFC page.</summary>
    public string ShowUfcKey { get; set; } = "PrevPage";
}

/// <summary>F-16C DED/NAV/RWR page options.</summary>
public class F16COptions
{
    /// <summary>MCDU key that shows the DED display.</summary>
    public string DedKey { get; set; } = "PrevPage";

    /// <summary>MCDU key that shows the NAV display.</summary>
    public string NavKey { get; set; } = "NextPage";

    /// <summary>MCDU key that shows the RWR display.</summary>
    public string RwrKey { get; set; } = "LineSelectRight1";
}

/// <summary>F-14B RIO CAP / radio page options.</summary>
public class F14Options
{
    /// <summary>MCDU key that shows the RIO CAP display.</summary>
    public string RioKey { get; set; } = "PrevPage";

    /// <summary>MCDU key that shows the radio display.</summary>
    public string RadioKey { get; set; } = "NextPage";
}
