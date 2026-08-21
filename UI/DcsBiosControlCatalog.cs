using System;
using System.Collections.Generic;
using System.Linq;
using DCS_BIOS.Json;
using DCS_BIOS.Serialized;

namespace WCtrlDcsBiosBridge.UI;

/// <summary>
/// One DCS-BIOS control the user can bind, as shown in the picker.
/// </summary>
public sealed record ControlChoice(string Identifier, string Description, string Category, int MaxValue, bool IsLamp)
{
    /// <summary>What the picker shows: the human description first, the identifier to confirm it.</summary>
    public string Display => $"{Description} ({Identifier})";

    /// <summary>
    /// Whether a plain on/off test is not enough. A lamp is 0 or 1; a selector or a gauge
    /// needs the user to say which position lights the LED.
    /// </summary>
    public bool NeedsCondition => MaxValue > 1;

    public override string ToString() => Display;
}

/// <summary>
/// Turns a module's DCS-BIOS controls into the list the LED editor offers. Separate from the
/// panel so the rules can be tested without a window.
/// </summary>
public static class DcsBiosControlCatalog
{
    /// <summary>The DCS-BIOS control_type that marks an indicator lamp.</summary>
    private const string LampControlType = "led";

    /// <summary>
    /// The controls a LED can follow: those with an integer output. String outputs (CDU
    /// lines, counters) have no on/off reading, so they are left out entirely.
    /// </summary>
    public static IReadOnlyList<ControlChoice> Bindable(IEnumerable<DCSBIOSControl>? controls)
    {
        if (controls is null) return Array.Empty<ControlChoice>();

        return controls
            .Where(c => c?.Outputs != null && c.Outputs.Any(o => o.OutputDataType == DCSBiosOutputType.IntegerType))
            .Select(c => new ControlChoice(
                c.Identifier ?? string.Empty,
                string.IsNullOrWhiteSpace(c.Description) ? c.Identifier ?? string.Empty : c.Description,
                c.Category ?? string.Empty,
                c.Outputs.First(o => o.OutputDataType == DCSBiosOutputType.IntegerType).MaxValue,
                string.Equals(c.ControlType, LampControlType, StringComparison.OrdinalIgnoreCase)))
            .Where(c => c.Identifier.Length > 0)
            .OrderBy(c => c.Category, StringComparer.OrdinalIgnoreCase)
            .ThenBy(c => c.Description, StringComparer.OrdinalIgnoreCase)
            .ToList();
    }

    /// <summary>
    /// What the picker shows: the module's indicator lamps, or every integer output when the
    /// user asked for all of them — or when the module declares no lamp at all, since an
    /// empty list would just look broken.
    /// </summary>
    public static IReadOnlyList<ControlChoice> Visible(IReadOnlyList<ControlChoice> all, bool showEverything)
    {
        if (showEverything) return all;

        var lamps = all.Where(c => c.IsLamp).ToList();
        return lamps.Count > 0 ? lamps : all;
    }

    /// <summary>Whether the lamp filter would hide nothing, because the module declares none.</summary>
    public static bool HasLamps(IReadOnlyList<ControlChoice> all) => all.Any(c => c.IsLamp);
}
