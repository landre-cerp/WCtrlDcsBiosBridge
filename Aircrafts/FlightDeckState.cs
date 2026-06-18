namespace WCtrlDcsBiosBridge.Aircrafts;

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
    public int? PltCourse { get; set; }
    public int? CplCourse { get; set; }


    /// <summary>
    /// Barometric pressure in inHg * 100 (FCU convention, e.g. 2992 for 29.92 inHg).
    /// </summary>
    public int? BaroPressure { get; set; }

    public bool? GearLeftDown { get; set; }
    public bool? GearNoseDown { get; set; }
    public bool? GearRightDown { get; set; }

    public bool? LedAutoBrkLoOn { get; set; }
    public bool? LedAutoBrkMedOn { get; set; }
    public bool? LedAutoBrkMaxOn { get; set; }

    public bool? LedAutoBrkLoDecel { get; set; }
    public bool? LedAutoBrkMedDecel { get; set; }
    public bool? LedAutoBrkMaxDecel { get; set; }

    public bool? LedTerrOnNd { get; set; }

    public bool? LedBrkFanOn { get; set; }
    public bool? LedBrkFanHot { get; set; }

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
    /// Chronograph display as "MMSS" digits (e.g. "1234" for 12:34).
    /// </summary>
    public string? ClockChrono { get; set; }

    /// <summary>
    /// UTC clock display as "HHMMSS" digits (e.g. "123456" for 12:34:56).
    /// </summary>
    public string? ClockUtcTime { get; set; }

    /// <summary>
    /// Elapsed-time display as "HHMM" digits (e.g. "1234" for 12:34).
    /// </summary>
    public string? ClockElapsedTime { get; set; }

}
