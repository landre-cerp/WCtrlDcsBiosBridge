using System.Collections.Concurrent;
using WCtrlDcsBiosBridge.Devices.Frontpanels;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// Semantic, device-agnostic flight deck state. Aircraft listeners write the values
/// they can provide; frontpanel renderers read whatever their device can show.
/// A null value means "this aircraft does not provide it".
/// </summary>
internal class FlightDeckState
{
    // User LED bindings bypass the semantic properties below: the listener evaluates the
    // user's condition when the DCS-BIOS value arrives and drops the resulting on/off here,
    // keyed by device family and LED id. Concurrent because DCS-BIOS handlers write on the
    // receive thread while the frontpanel hub reads on its 100 ms render timer.
    private readonly ConcurrentDictionary<string, bool> _userLeds = new();

    private static string UserLedKey(LedDeviceFamily family, string ledId) =>
        $"{family}:{ledId.ToUpperInvariant()}";

    /// <summary>Records the evaluated state of a user-bound LED.</summary>
    public void SetUserLed(LedDeviceFamily family, string ledId, bool on) =>
        _userLeds[UserLedKey(family, ledId)] = on;

    /// <summary>
    /// Reads a user-bound LED. False when the user did not bind it — the renderer then
    /// leaves that LED to its built-in mapping.
    /// </summary>
    public bool TryGetUserLed(LedDeviceFamily family, string ledId, out bool on) =>
        _userLeds.TryGetValue(UserLedKey(family, ledId), out on);

    /// <summary>Whether any user binding writes here, so renderers can skip the overlay.</summary>
    public bool HasUserLeds => !_userLeds.IsEmpty;

    /// <summary>
    /// Writes one of the LED signals by name, for the declared per-aircraft defaults. The
    /// properties below stay the way listeners and renderers read them; this is only the door
    /// that a table-driven default comes in through.
    /// </summary>
    public void SetSignal(FlightDeckSignal signal, bool on)
    {
        switch (signal)
        {
            case FlightDeckSignal.GearLeftDown: GearLeftDown = on; break;
            case FlightDeckSignal.GearNoseDown: GearNoseDown = on; break;
            case FlightDeckSignal.GearRightDown: GearRightDown = on; break;
            case FlightDeckSignal.GearWarning: GearWarning = on; break;
            case FlightDeckSignal.AutoBrkLoOn: LedAutoBrkLoOn = on; break;
            case FlightDeckSignal.AutoBrkMedOn: LedAutoBrkMedOn = on; break;
            case FlightDeckSignal.AutoBrkMaxOn: LedAutoBrkMaxOn = on; break;
            case FlightDeckSignal.AutoBrkLoDecel: LedAutoBrkLoDecel = on; break;
            case FlightDeckSignal.AutoBrkMedDecel: LedAutoBrkMedDecel = on; break;
            case FlightDeckSignal.AutoBrkMaxDecel: LedAutoBrkMaxDecel = on; break;
            case FlightDeckSignal.TerrOnNd: LedTerrOnNd = on; break;
            case FlightDeckSignal.BrkFanOn: LedBrkFanOn = on; break;
            case FlightDeckSignal.BrkFanHot: LedBrkFanHot = on; break;
        }
    }

    /// <summary>
    /// Reads a LED signal. Null means this aircraft does not publish it, which renderers show
    /// as off.
    /// </summary>
    public bool? GetSignal(FlightDeckSignal signal) => signal switch
    {
        FlightDeckSignal.GearLeftDown => GearLeftDown,
        FlightDeckSignal.GearNoseDown => GearNoseDown,
        FlightDeckSignal.GearRightDown => GearRightDown,
        FlightDeckSignal.GearWarning => GearWarning,
        FlightDeckSignal.AutoBrkLoOn => LedAutoBrkLoOn,
        FlightDeckSignal.AutoBrkMedOn => LedAutoBrkMedOn,
        FlightDeckSignal.AutoBrkMaxOn => LedAutoBrkMaxOn,
        FlightDeckSignal.AutoBrkLoDecel => LedAutoBrkLoDecel,
        FlightDeckSignal.AutoBrkMedDecel => LedAutoBrkMedDecel,
        FlightDeckSignal.AutoBrkMaxDecel => LedAutoBrkMaxDecel,
        FlightDeckSignal.TerrOnNd => LedTerrOnNd,
        FlightDeckSignal.BrkFanOn => LedBrkFanOn,
        FlightDeckSignal.BrkFanHot => LedBrkFanHot,
        _ => null,
    };


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
