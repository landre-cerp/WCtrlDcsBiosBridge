namespace WWCduDcsBiosBridge.Aircrafts;

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
    Func<AircraftCreationContext, AircraftListener> Create);

/// <summary>
/// A selectable menu entry derived from the registry. Public for WPF binding.
/// </summary>
public sealed record AircraftMenuEntry(string Label, AircraftSelection Selection);

internal static class AircraftRegistry
{
    public static readonly AircraftDescriptor A10C = new(
        5, "A-10C", "A-10C.json", "resources/a10c-font-21x31.json", false,
        c => new A10C_Listener(c.Options));

    public static readonly AircraftDescriptor AH64D = new(
        46, "AH-64D", "AH-64D.json", "resources/ah64d-font-21x31.json", false,
        c => new AH64D_Listener(c.Options));

    public static readonly AircraftDescriptor FA18C = new(
        20, "F/A-18C", "FA-18C_hornet.json", "resources/a10c-font-21x31.json", false,
        c => new FA18C_Listener(c.Options));

    public static readonly AircraftDescriptor CH47 = new(
        50, "CH-47F", "CH-47F.json", "resources/ch47f-font-21x31.json", true,
        c => new CH47F_Listener(c.Options, c.IsPilot, c.Ch47SwitchWithSeat));

    public static readonly AircraftDescriptor F15E = new(
        44, "F-15E", "F-15E.json", "resources/a10c-font-21x31.json", false,
        c => new F15E_Listener(c.Options));

    public static readonly AircraftDescriptor M2000C = new(
        27, "M-2000C", "M-2000C.json", "resources/ah64d-font-21x31.json", false,
        c => new M2000C_Listener(c.Options));

    public static readonly AircraftDescriptor F16C = new(
        17, "F-16C", "F-16C_50.json", "resources/ah64d-font-21x31.json", false,
        c => new F16C_Listener(c.Options));

    public static readonly AircraftDescriptor OH58D = new(
        49, "OH-58D", "OH-58D.json", "resources/oh58d-font-21x31.json", false,
        c => new OH58D_Listener(c.Options));

    /// <summary>
    /// Registry order is menu order.
    /// </summary>
    public static readonly IReadOnlyList<AircraftDescriptor> All = new[]
    {
        A10C, AH64D, FA18C, CH47, F15E, M2000C, F16C, OH58D,
    };

    public static AircraftDescriptor Find(int moduleId) =>
        All.FirstOrDefault(d => d.ModuleId == moduleId)
        ?? throw new NotSupportedException($"Aircraft {moduleId} not supported");

    public static IEnumerable<string> ExpectedJsonFiles => All.Select(d => d.JsonFile);

    /// <summary>
    /// Builds the selectable entries shown on the CDU menu and the WPF panel.
    /// Seat-selection aircraft (CH-47F) contribute a single entry when the CDU
    /// display switches with the seat (single-CDU setups), otherwise a PLT/CPLT pair.
    /// </summary>
    public static IReadOnlyList<AircraftMenuEntry> BuildMenuEntries(bool ch47SwitchWithSeat)
    {
        var entries = new List<AircraftMenuEntry>();
        foreach (var descriptor in All)
        {
            if (descriptor.HasSeatSelection && !ch47SwitchWithSeat)
            {
                entries.Add(new AircraftMenuEntry($"{descriptor.DisplayName} (PLT)", new AircraftSelection(descriptor.ModuleId, true)));
                entries.Add(new AircraftMenuEntry($"{descriptor.DisplayName} (CPLT)", new AircraftSelection(descriptor.ModuleId, false)));
            }
            else
            {
                entries.Add(new AircraftMenuEntry(descriptor.DisplayName, new AircraftSelection(descriptor.ModuleId, true)));
            }
        }
        return entries;
    }
}
