using System;
using System.Collections.Generic;
using System.Linq;
using WCtrlDcsBiosBridge.Devices.Frontpanels;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// The device-agnostic LED signals an aircraft can publish. A listener says "the left gear is
/// down"; each panel family decides which of its LEDs shows that, so one declaration lights
/// the right lamp on every panel that has one.
/// </summary>
public enum FlightDeckSignal
{
    GearLeftDown,
    GearNoseDown,
    GearRightDown,
    GearWarning,
    AutoBrkLoOn,
    AutoBrkMedOn,
    AutoBrkMaxOn,
    AutoBrkLoDecel,
    AutoBrkMedDecel,
    AutoBrkMaxDecel,
    TerrOnNd,
    BrkFanOn,
    BrkFanHot,
}

/// <summary>
/// A signal this aircraft reads off DCS-BIOS controls: lit when any of them is non-zero.
/// One control is the usual case; several cover a lamp that answers to a set of conditions,
/// like a gear warning that lights when any leg is not locked down.
/// </summary>
public sealed record SignalDefault(FlightDeckSignal Signal, IReadOnlyList<string> Controls)
{
    public SignalDefault(FlightDeckSignal signal, params string[] controls)
        : this(signal, (IReadOnlyList<string>)controls) { }
}

/// <summary>An MCDU annunciator this aircraft drives straight from one DCS-BIOS control.</summary>
public sealed record McduLedDefault(McduLed Led, string Control);

/// <summary>
/// What the editor shows for a LED nobody has bound: the control it already follows, or a note
/// naming what the code works it out from. Both null means the LED is idle on this aircraft.
/// </summary>
public sealed record LedDefaultInfo(IReadOnlyList<string>? Controls, string? ComputedFrom)
{
    public static readonly LedDefaultInfo None = new(null, null);

    /// <summary>A signal read off these controls.</summary>
    public static LedDefaultInfo From(IReadOnlyList<string> controls) => new(controls, null);

    public static LedDefaultInfo From(string control) => new(new[] { control }, null);

    /// <summary>A signal the listener works out, described by what it derives it from.</summary>
    public static LedDefaultInfo Computed(string from) => new(null, from);

    public bool Exists => Controls is { Count: > 0 } || ComputedFrom != null;

    /// <summary>
    /// Names the controls compactly enough for a placeholder: the first one, and how many
    /// others join it.
    /// </summary>
    public string? Describe()
    {
        if (Controls is not { Count: > 0 }) return null;
        return Controls.Count == 1 ? Controls[0] : $"{Controls[0]} +{Controls.Count - 1}";
    }
}

/// <summary>
/// One aircraft's built-in LED behaviour, declared rather than buried in a lambda, so that the
/// listener that applies it and the editor that displays it read the same table and cannot
/// drift apart.
/// </summary>
public sealed class AircraftLedDefaults
{
    public IReadOnlyList<SignalDefault> Signals { get; init; } = Array.Empty<SignalDefault>();

    public IReadOnlyList<McduLedDefault> McduLeds { get; init; } = Array.Empty<McduLedDefault>();

    /// <summary>
    /// Signals the listener derives in code because no single control expresses them — an OR of
    /// three lights, a bitfield off a raw register. The value names what it is worked out from.
    /// </summary>
    public IReadOnlyDictionary<FlightDeckSignal, string> ComputedSignals { get; init; } =
        new Dictionary<FlightDeckSignal, string>();

    /// <summary>MCDU annunciators derived in code, as <see cref="ComputedSignals"/>.</summary>
    public IReadOnlyDictionary<McduLed, string> ComputedMcduLeds { get; init; } =
        new Dictionary<McduLed, string>();

    public static readonly AircraftLedDefaults None = new();

    public LedDefaultInfo For(FlightDeckSignal signal)
    {
        var direct = Signals.FirstOrDefault(s => s.Signal == signal);
        if (direct != null) return LedDefaultInfo.From(direct.Controls);

        return ComputedSignals.TryGetValue(signal, out var from)
            ? LedDefaultInfo.Computed(from)
            : LedDefaultInfo.None;
    }

    public LedDefaultInfo For(McduLed led)
    {
        var direct = McduLeds.FirstOrDefault(l => l.Led == led);
        if (direct != null) return LedDefaultInfo.From(direct.Control);

        return ComputedMcduLeds.TryGetValue(led, out var from)
            ? LedDefaultInfo.Computed(from)
            : LedDefaultInfo.None;
    }
}

/// <summary>
/// Every aircraft's built-in LED behaviour, in one table. The base listener registers these
/// on start; the LED editor shows them as each row's starting point, so a user who has bound
/// nothing still sees what the aircraft is driving, and clearing a binding puts it back.
/// </summary>
public static class LedDefaults
{
    private static readonly Dictionary<int, AircraftLedDefaults> _byModuleId = new()
    {
        // A-10C
        [5] = new AircraftLedDefaults
        {
            Signals = new[]
            {
                new SignalDefault(FlightDeckSignal.GearLeftDown, "GEAR_L_SAFE"),
                new SignalDefault(FlightDeckSignal.GearNoseDown, "GEAR_N_SAFE"),
                new SignalDefault(FlightDeckSignal.GearRightDown, "GEAR_R_SAFE"),
                new SignalDefault(FlightDeckSignal.GearWarning, "HANDLE_GEAR_WARNING"),
            },
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "MASTER_CAUTION"),
                new McduLedDefault(McduLed.Fm1, "GUN_READY"),
                new McduLedDefault(McduLed.Fm2, "CANOPY_UNLOCKED"),
                new McduLedDefault(McduLed.Ind, "NOSEWHEEL_STEERING"),
            },
        },

        // AH-64D
        [46] = new AircraftLedDefaults
        {
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "PLT_MASTER_CAUTION_L"),
                new McduLedDefault(McduLed.Ind, "PLT_MASTER_WARNING_L"),
            },
        },

        // F/A-18C
        [20] = new AircraftLedDefaults
        {
            Signals = new[]
            {
                new SignalDefault(FlightDeckSignal.GearLeftDown, "FLP_LG_LEFT_GEAR_LT"),
                new SignalDefault(FlightDeckSignal.GearNoseDown, "FLP_LG_NOSE_GEAR_LT"),
                new SignalDefault(FlightDeckSignal.GearRightDown, "FLP_LG_RIGHT_GEAR_LT"),
                new SignalDefault(FlightDeckSignal.GearWarning, "LANDING_GEAR_HANDLE_LT"),
                new SignalDefault(FlightDeckSignal.AutoBrkLoDecel, "FLP_LG_HALF_FLAPS_LT"),
                new SignalDefault(FlightDeckSignal.AutoBrkMedDecel, "FLP_LG_FULL_FLAPS_LT"),
            },
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "MASTER_CAUTION_LT"),
            },
        },

        // CH-47F
        [50] = new AircraftLedDefaults
        {
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "PLT_MASTER_CAUTION_LIGHT"),
            },
        },

        // M-2000C — every LED comes off a raw lighting register, so none of them is one control.
        [27] = new AircraftLedDefaults
        {
            ComputedSignals = new Dictionary<FlightDeckSignal, string>
            {
                [FlightDeckSignal.GearLeftDown] = "gear configuration bits",
                [FlightDeckSignal.GearNoseDown] = "gear configuration bits",
                [FlightDeckSignal.GearRightDown] = "gear configuration bits",
                [FlightDeckSignal.GearWarning] = "gear lever and configuration bits",
            },
            ComputedMcduLeds = new Dictionary<McduLed, string>
            {
                [McduLed.Fail] = "PCN light register (UNI)",
                [McduLed.Fm1] = "PCN light register (ALN)",
                [McduLed.Fm2] = "PCN light register (SEC)",
                [McduLed.Fm] = "PCN light register (NDEG)",
                [McduLed.Ind] = "PCN light register (MIP)",
                [McduLed.Rdy] = "PCN light register (PRET)",
            },
        },

        // F-16C
        [17] = new AircraftLedDefaults
        {
            Signals = new[]
            {
                new SignalDefault(FlightDeckSignal.GearLeftDown, "LIGHT_GEAR_L"),
                new SignalDefault(FlightDeckSignal.GearNoseDown, "LIGHT_GEAR_N"),
                new SignalDefault(FlightDeckSignal.GearRightDown, "LIGHT_GEAR_R"),
                new SignalDefault(FlightDeckSignal.GearWarning, "LIGHT_GEAR_WARN"),
            },
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "LIGHT_MASTER_CAUTION"),
            },
        },

        // UH-1H
        [38] = new AircraftLedDefaults
        {
            McduLeds = new[]
            {
                new McduLedDefault(McduLed.Fail, "MASTER_CAUTION_IND"),
            },
        },

        // F-14B — the warning lights when any leg reports itself not locked down.
        [16] = new AircraftLedDefaults
        {
            Signals = new[]
            {
                new SignalDefault(FlightDeckSignal.GearLeftDown, "PLT_GEAR_L_IND_L"),
                new SignalDefault(FlightDeckSignal.GearNoseDown, "PLT_GEAR_NOSE_IND_L"),
                new SignalDefault(FlightDeckSignal.GearRightDown, "PLT_GEAR_R_IND_L"),
                new SignalDefault(FlightDeckSignal.GearWarning,
                    "PLT_GEAR_L_OFF_L", "PLT_GEAR_NOSE_OFF_L", "PLT_GEAR_R_OFF_L"),
            },

        },
    };

    /// <summary>What <paramref name="descriptor"/>'s aircraft lights on its own.</summary>
    internal static AircraftLedDefaults For(AircraftDescriptor descriptor) =>
        _byModuleId.GetValueOrDefault(descriptor.ModuleId, AircraftLedDefaults.None);

    /// <summary>What the aircraft with this registry display name lights on its own.</summary>
    internal static AircraftLedDefaults ForDisplayName(string displayName)
    {
        var descriptor = AircraftRegistry.All.FirstOrDefault(d =>
            string.Equals(d.DisplayName, displayName, StringComparison.Ordinal));

        return descriptor is null ? AircraftLedDefaults.None : For(descriptor);
    }
}
