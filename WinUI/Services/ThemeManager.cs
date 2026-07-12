using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Media;
using Microsoft.Win32;
using WCtrlDcsBiosBridge.Config;
using Windows.UI;

namespace WCtrlDcsBiosBridge.Services;

public static class ThemeManager
{
    /// <summary>The Fluent light/dark theme matching the last applied preference — new
    /// windows read this to set their RequestedTheme so native controls (Pivot, Button,
    /// CheckBox...) render with matching chrome instead of always following the OS theme.</summary>
    public static ElementTheme CurrentElementTheme { get; private set; } = ElementTheme.Default;

    public static ElementTheme ToElementTheme(ThemePreference preference) => preference switch
    {
        ThemePreference.Light => ElementTheme.Light,
        ThemePreference.Dark => ElementTheme.Dark,
        ThemePreference.DCS => ElementTheme.Dark,
        _ => ElementTheme.Default
    };

    public static bool IsSystemDark()
    {
        try
        {
            var val = Registry.GetValue(
                @"HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Themes\Personalize",
                "AppsUseLightTheme", 1);
            return val is int i && i == 0;
        }
        catch { return false; }
    }

    private static Color C(byte a, byte r, byte g, byte b) => Color.FromArgb(a, r, g, b);
    private static Color C(byte r, byte g, byte b) => Color.FromArgb(0xFF, r, g, b);

    // "Dark" now uses the former DCS palette's colors — Laurent asked to fold the
    // separate DCS theme into the default Dark theme rather than keep three themes.
    private static readonly Dictionary<string, Color> DarkPalette = new()
    {
        ["WindowBackground"] = C(0x0D, 0x0E, 0x10),
        ["CardBorderBrush"] = C(0x38, 0x3C, 0x48),
        ["SubtleTextForeground"] = C(0x8C, 0x9A, 0xB0),
        ["HelpTextForeground"] = C(0x8C, 0x9A, 0xB0),
        ["StepBorderBrush"] = C(0x38, 0x3C, 0x48),
        ["StepForeground"] = C(0x8C, 0x9A, 0xB0),
        ["UpdateBannerBackground"] = C(0x2A, 0x1C, 0x00),
        ["UpdateBannerBorderBrush"] = C(0x7A, 0x51, 0x00),
        ["UpdateBannerForeground"] = C(0xF5, 0xA6, 0x23),
        ["CardConnectedBorderBrush"] = C(0xE8, 0x93, 0x0A),
        ["CardConnectedBgBrush"] = C(0x18, 0xE8, 0x93, 0x0A),
        ["CardConnectedDotBrush"] = C(0xF5, 0xA6, 0x23),
    };

    private static readonly Dictionary<string, Color> LightPalette = new()
    {
        ["WindowBackground"] = C(0xF3, 0xF3, 0xF3),
        ["CardBorderBrush"] = C(0xD1, 0xD5, 0xDB),
        ["SubtleTextForeground"] = C(0x77, 0x77, 0x77),
        ["HelpTextForeground"] = C(0x55, 0x55, 0x55),
        ["StepBorderBrush"] = C(0xBB, 0xBB, 0xBB),
        ["StepForeground"] = C(0x88, 0x88, 0x88),
        ["UpdateBannerBackground"] = C(0xFD, 0xF5, 0xD7),
        ["UpdateBannerBorderBrush"] = C(0xE6, 0xC2, 0x00),
        ["UpdateBannerForeground"] = C(0x66, 0x44, 0x00),
        ["CardConnectedBorderBrush"] = C(0x2E, 0x7D, 0x32),
        ["CardConnectedBgBrush"] = C(0x18, 0x2E, 0x7D, 0x32),
        ["CardConnectedDotBrush"] = C(0x43, 0xA0, 0x47),
    };

    public static void Apply(ThemePreference preference)
    {
        // DCS folded into Dark: any persisted ThemePreference.DCS value (from earlier
        // testing) still resolves to the same palette as Dark.
        var isLight = preference == ThemePreference.Light
            || (preference == ThemePreference.System && !IsSystemDark());

        var palette = isLight ? LightPalette : DarkPalette;

        var resources = Application.Current.Resources;
        foreach (var (key, color) in palette)
        {
            if (resources.TryGetValue(key, out var value) && value is SolidColorBrush brush)
            {
                brush.Color = color;
            }
        }

        CurrentElementTheme = ToElementTheme(preference);
    }
}
