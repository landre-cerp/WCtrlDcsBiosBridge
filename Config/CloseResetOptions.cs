namespace WWCduDcsBiosBridge.Config;

/// <summary>
/// Controls what the app resets on connected devices when closing.
/// Screen content is always cleared; only backlight and marker (LED) behaviour
/// is user-configurable.
/// </summary>
internal record CloseResetOptions(bool Backlight, bool Markers)
{
    /// <summary>
    /// Target brightness percent for CDU Cleanup calls (0-100).
    /// 0 % when Backlight is on, 80 % otherwise (visible but blank).
    /// </summary>
    public int BrightnessPercent => Backlight ? 0 : 80;

    /// <summary>
    /// Target brightness byte for IFrontpanel.SetBrightness (0-255).
    /// </summary>
    public byte BrightnessByte => (byte)(BrightnessPercent * 255 / 100);

    public static CloseResetOptions From(UserOptions opts) =>
        new(opts.CloseResetBacklight, opts.CloseResetMarkers);
}
