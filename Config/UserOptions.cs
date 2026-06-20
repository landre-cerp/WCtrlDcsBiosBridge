using WCtrlDcsBiosBridge.Config;

/// <summary>
/// User-configurable options for the application and aircraft-specific settings.
/// These options are persisted to useroptions.json.
/// </summary>
public class UserOptions
{
    /// <summary>
    /// Gets or sets whether the A10C display should be aligned to the bottom of the screen.
    /// </summary>
    public bool DisplayBottomAligned { get; set; }

    /// <summary>
    /// Gets or sets whether the A10C CMS should be displayed.
    /// </summary>
    public bool DisplayCMS { get; set; }

    /// <summary>
    /// Gets or sets whether lighting management should be disabled (for SimApp Pro users).
    /// </summary>
    public bool DisableLightingManagement { get; set; }

    /// <summary>
    /// Gets or sets whether the bridge should automatically start when conditions are met.
    /// </summary>
    public bool AutoStart { get; set; }

    /// <summary>
    /// Gets or sets whether the application window should be minimized when the bridge starts.
    /// When enabled, the main window is automatically minimized after successful bridge startup.
    /// </summary>
    public bool MinimizeOnStart { get; set; }

    /// <summary>
    /// Gets or sets the MCDU key used to switch to the next page.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "NextPage", "LineSelectRight1").
    /// </summary>
    public string NextPageKey { get; set; } = "NextPage";

    /// <summary>
    /// Gets or sets the MCDU key used to switch to the previous page.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "PrevPage", "LineSelectLeft1").
    /// </summary>
    public string PrevPageKey { get; set; } = "PrevPage";

    /// <summary>
    /// Gets or sets the MCDU key used to show the F-16C DED display.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "PrevPage", "LeftArrow").
    /// </summary>
    public string F16CPrevDisplayKey { get; set; } = "PrevPage";

    /// <summary>
    /// Gets or sets the MCDU key used to show the F-16C NAV display.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "NextPage", "RightArrow").
    /// </summary>
    public string F16CNextDisplayKey { get; set; } = "NextPage";

    /// <summary>
    /// Gets or sets the MCDU key used to show the F-16C RWR display.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "F1").
    /// </summary>
    public string F16CRwrDisplayKey { get; set; } = "LineSelectRight1";

    /// <summary>
    /// Gets or sets the MCDU key used to show the F-14B RIO CAP display.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "PrevPage").
    /// </summary>
    public string F14RioDisplayKey { get; set; } = "PrevPage";

    /// <summary>
    /// Gets or sets the MCDU key used to show the F-14B radio display.
    /// Value must be a valid <see cref="WwDevicesDotNet.Key"/> enum name (e.g., "NextPage").
    /// </summary>
    public string F14RadioDisplayKey { get; set; } = "NextPage";

    /// <summary>
    /// Gets or sets the preferred application theme (System, Light, or Dark).
    /// </summary>
    public ThemePreference Theme { get; set; } = ThemePreference.System;

    // ── On-close device reset ────────────────────────────────────────────────
    // Only applies when DisableLightingManagement is false.
    // Screen content is always cleared on close.

    /// <summary>
    /// Set all device backlights to 0 % when the application closes.
    /// When false, backlight stays at 80 % (blank but visible).
    /// </summary>
    public bool CloseResetBacklight { get; set; } = true;

    /// <summary>
    /// Turn off all button/panel LEDs (markers) when the application closes.
    /// </summary>
    public bool CloseResetMarkers { get; set; } = true;
}
