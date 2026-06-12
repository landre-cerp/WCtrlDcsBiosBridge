namespace WWCduDcsBiosBridge.Aircrafts;

/// <summary>
/// Semantic, device-agnostic flight deck state. Aircraft listeners write the values
/// they can provide; frontpanel renderers read whatever their device can show.
/// A null value means "this aircraft does not provide it".
/// </summary>
internal class FlightDeckState
{
    public int? Speed { get; set; }
    public int? Heading { get; set; }
    public int? Altitude { get; set; }
    public int? VerticalSpeed { get; set; }

    /// <summary>
    /// Barometric pressure in inHg * 100 (FCU convention, e.g. 2992 for 29.92 inHg).
    /// </summary>
    public int? BaroPressure { get; set; }

    public bool? GearLeftDown { get; set; }
    public bool? GearNoseDown { get; set; }
    public bool? GearRightDown { get; set; }

    /// <summary>Gear handle warning light (red).</summary>
    public bool? GearWarning { get; set; }

    /// <summary>
    /// Cockpit console brightness, 0-255. Null when the aircraft does not drive it
    /// (or lighting management is disabled); renderers then keep their family default.
    /// </summary>
    public byte? ConsoleBrightness { get; set; }

    /// <summary>
    /// 7 Segment LCD display brightness, 0-255. Null when the aircraft does not drive it
    /// (or lighting management is disabled); renderers then keep their family default.
    /// </summary>
    public int SegmentBrightnessPercent { get; set; } = 100;

    /// <summary>
    /// AGP32 chrono display as "MMSS" digits (device renders it as "88:88").
    /// Example: "1234" for 12:34.
    /// </summary>
    public string? Agp32Chrono { get; set; }

    /// <summary>
    /// AGP32 UTC time display as "HHMMSS" digits (device renders it as "88:88:88").
    /// Example: "123456" for 12:34:56.
    /// </summary>
    public string? Agp32UtcTime { get; set; }

    /// <summary>
    /// AGP32 ET display as "HHMM" digits (device renders it as "88:88").
    /// Example: "1234" for 12:34.
    /// </summary>
    public string? Agp32Et { get; set; }

}
