using WCtrlDcsBiosBridge.Aircrafts.UH1H;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// Everything the bridge needs to know to create a listener for an aircraft.
/// </summary>
internal sealed record AircraftCreationContext(
    UserOptions Options,
    bool IsPilot,
    bool Ch47SwitchWithSeat);

/// <summary>
/// Self-describing aircraft registration: DCS-BIOS module id, names, resources
/// and the listener factory. Adding an aircraft means adding one descriptor to
/// <see cref="AircraftRegistry.All"/> (plus the listener class itself) — the
/// factory, the CDU menu, the WPF selection panel and the DCS-BIOS json check
/// all derive from the registry.
/// </summary>
internal sealed record AircraftDescriptor(
    int ModuleId,
    string DisplayName,
    string JsonFile,
    string FontFile,
    bool HasSeatSelection,
    string[] DcsBiosNames,
    Func<AircraftCreationContext, AircraftListener> Create);

/// <summary>
/// The user's resolved aircraft choice: which module, and which seat (pilot vs
/// copilot) for dual-seat aircraft.
/// </summary>
public sealed record AircraftSelection(int AircraftId, bool IsPilot);

internal static class AircraftRegistry
{
    public static readonly AircraftDescriptor A10C = new(
        5, "A-10C", "A-10C.json", "resources/a10c-font-21x31.json", false,
        new[] { "A-10C", "A-10C_2" },
        c => new A10C_Listener(c.Options));

    public static readonly AircraftDescriptor AH64D = new(
        46, "AH-64D", "AH-64D.json", "resources/ah64d-font-21x31.json", false,
        new[] { "AH-64D" },
        c => new AH64D_Listener(c.Options));

    public static readonly AircraftDescriptor FA18C = new(
        20, "F/A-18C", "FA-18C_hornet.json", "resources/a10c-font-21x31.json", false,
        new[] { "FA-18C" },
        c => new FA18C_Listener(c.Options));

    public static readonly AircraftDescriptor CH47 = new(
        50, "CH-47F", "CH-47F.json", "resources/ch47f-font-21x31.json", true,
        new[] { "CH-47F" },
        c => new CH47F_Listener(c.Options, c.IsPilot, c.Ch47SwitchWithSeat));

    public static readonly AircraftDescriptor F15E = new(
        44, "F-15E", "F-15E.json", "resources/a10c-font-21x31.json", false,
        new[] { "F-15E" },
        c => new F15E_Listener(c.Options));

    public static readonly AircraftDescriptor M2000C = new(
        27, "M-2000C", "M-2000C.json", "resources/ah64d-font-21x31.json", false,
        new[] { "M-2000C" },
        c => new M2000C_Listener(c.Options));

    public static readonly AircraftDescriptor F16C = new(
        17, "F-16C", "F-16C_50.json", "resources/ah64d-font-21x31.json", false,
        new[] { "F-16C" },
        c => new F16C_Listener(c.Options));

    public static readonly AircraftDescriptor OH58D = new(
        49, "OH-58D", "OH-58D.json", "resources/oh58d-font-21x31.json", false,
        new[] { "OH58D" },
        c => new OH58D_Listener(c.Options));

    public static readonly AircraftDescriptor UH1H = new(
        38, "UH-1H", "UH-1H.json", "resources/a10c-font-21x31.json", false,
        new[] { "UH-1H" },
        c => new UH1H_Listener(c.Options));

    /// <summary>
    /// Registry order is menu order.
    /// </summary>
    public static readonly IReadOnlyList<AircraftDescriptor> All = new[]
    {
        A10C, AH64D, FA18C, CH47, F15E, M2000C, F16C, OH58D, UH1H
    };

    public static AircraftDescriptor Find(int moduleId) =>
        All.FirstOrDefault(d => d.ModuleId == moduleId)
        ?? throw new NotSupportedException($"Aircraft {moduleId} not supported");

    /// <summary>
    /// Looks up a descriptor by the DCS-BIOS aircraft name sent via
    /// <c>MetadataStart/_ACFT_NAME</c>. Matches when the detected name
    /// starts with a registered DCS-BIOS name (case-insensitive), so
    /// e.g. "AH-64D_BLK_II" matches registered name "AH-64D".
    /// Returns <c>null</c> when no match is found.
    /// </summary>
    public static AircraftDescriptor? FindByDcsBiosName(string dcsBiosName)
    {
        if (string.IsNullOrEmpty(dcsBiosName)) return null;
        return All.FirstOrDefault(d =>
            d.DcsBiosNames.Any(n => dcsBiosName.StartsWith(n, StringComparison.OrdinalIgnoreCase)));
    }

    public static IEnumerable<string> ExpectedJsonFiles => All.Select(d => d.JsonFile);
}
