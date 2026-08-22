using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using WCtrlDcsBiosBridge.Devices.Frontpanels;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// How a DCS-BIOS value decides whether the bound LED is lit.
/// </summary>
public enum LedCondition
{
    /// <summary>Lit when the value differs from <see cref="LedBinding.Value"/>. With the
    /// default threshold of 0 this is the plain indicator-lamp test: lit when not zero.</summary>
    NotEquals,

    /// <summary>Lit when the value equals <see cref="LedBinding.Value"/>.</summary>
    Equals,

    /// <summary>Lit when the value is greater than or equal to <see cref="LedBinding.Value"/>.</summary>
    AtLeast,

    /// <summary>Lit when the value is less than or equal to <see cref="LedBinding.Value"/>.</summary>
    AtMost,
}

/// <summary>
/// Reads the operator by name, and accepts the "NotZero" of files written before the
/// operator gained a threshold — mapping it to <see cref="LedCondition.NotEquals"/>, which
/// with the threshold those files also carry (0) is the same test. Without this one alias a
/// single old entry would fail the whole file's parse.
/// </summary>
public sealed class LedConditionConverter : JsonConverter<LedCondition>
{
    private const string LegacyNotZero = "NotZero";

    public override LedCondition Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
    {
        var name = reader.GetString();

        if (string.Equals(name, LegacyNotZero, StringComparison.OrdinalIgnoreCase))
            return LedCondition.NotEquals;

        return Enum.TryParse<LedCondition>(name, ignoreCase: true, out var parsed)
            ? parsed
            : throw new JsonException($"Unknown LED condition '{name}'.");
    }

    public override void Write(Utf8JsonWriter writer, LedCondition value, JsonSerializerOptions options) =>
        writer.WriteStringValue(value.ToString());
}

/// <summary>
/// One user-defined LED binding: on this aircraft, this LED of this device family follows
/// this DCS-BIOS control.
/// </summary>
public sealed class LedBinding
{
    /// <summary>The aircraft's registry display name, e.g. "F/A-18C".</summary>
    public string Aircraft { get; set; } = string.Empty;

    /// <summary>The device family the LED belongs to.</summary>
    public LedDeviceFamily Device { get; set; }

    /// <summary>The LED id, as listed by <see cref="LedCatalog.For"/>.</summary>
    public string Led { get; set; } = string.Empty;

    /// <summary>The DCS-BIOS control identifier driving the LED.</summary>
    public string Control { get; set; } = string.Empty;

    /// <summary>The test applied to the control's value.</summary>
    public LedCondition Op { get; set; } = LedCondition.NotEquals;

    /// <summary>The threshold <see cref="Op"/> compares against.</summary>
    public uint Value { get; set; }

    /// <summary>Whether the LED should be lit for <paramref name="value"/>.</summary>
    public bool IsOn(uint value) => Op switch
    {
        LedCondition.NotEquals => value != Value,
        LedCondition.Equals => value == Value,
        LedCondition.AtLeast => value >= Value,
        LedCondition.AtMost => value <= Value,
        _ => false,
    };

    /// <summary>The (aircraft, device, LED) triple a binding is unique on.</summary>
    internal (string, LedDeviceFamily, string) Slot =>
        (Aircraft.ToUpperInvariant(), Device, Led.ToUpperInvariant());
}

/// <summary>
/// The contents of ledmappings.json: a flat list, so the file stays readable and shareable
/// and so an unknown entry can be dropped without disturbing its neighbours.
/// </summary>
public sealed class LedMappingFile
{
    /// <summary>Schema version, for future migrations.</summary>
    public int Version { get; set; } = CurrentVersion;

    public List<LedBinding> Bindings { get; set; } = new();

    public const int CurrentVersion = 1;

    /// <summary>The bindings for one aircraft, by registry display name.</summary>
    public IReadOnlyList<LedBinding> ForAircraft(string aircraftDisplayName) =>
        Bindings.Where(b => string.Equals(b.Aircraft, aircraftDisplayName, StringComparison.OrdinalIgnoreCase))
                .ToList();

    /// <summary>
    /// Drops entries that cannot be applied and collapses duplicates, reporting one message
    /// per dropped entry. A shared or hand-edited file is expected to contain the odd stale
    /// binding — after a DCS-BIOS module update, say — and one bad line must never cost the
    /// user the rest of the file.
    /// </summary>
    /// <param name="knownAircraft">Registry display names that exist in this build.</param>
    public (LedMappingFile Clean, IReadOnlyList<string> Warnings) Sanitize(IEnumerable<string> knownAircraft)
    {
        var known = new HashSet<string>(knownAircraft, StringComparer.OrdinalIgnoreCase);
        var warnings = new List<string>();
        var kept = new Dictionary<(string, LedDeviceFamily, string), LedBinding>();

        foreach (var binding in Bindings)
        {
            if (binding is null) continue;

            if (!known.Contains(binding.Aircraft))
            {
                warnings.Add($"Unknown aircraft '{binding.Aircraft}' — binding for LED {binding.Device}/{binding.Led} ignored.");
                continue;
            }

            if (LedCatalog.Find(binding.Device, binding.Led) is null)
            {
                warnings.Add($"Unknown LED '{binding.Led}' on {binding.Device} — binding for {binding.Aircraft} ignored.");
                continue;
            }

            if (string.IsNullOrWhiteSpace(binding.Control))
            {
                warnings.Add($"No DCS-BIOS control set for {binding.Aircraft} {binding.Device}/{binding.Led} — binding ignored.");
                continue;
            }

            binding.Control = binding.Control.Trim();

            // Last one wins: a hand-merged file can name the same LED twice, and silently
            // keeping the later entry matches what the editor would have produced.
            if (kept.ContainsKey(binding.Slot))
                warnings.Add($"Duplicate binding for {binding.Aircraft} {binding.Device}/{binding.Led} — the last one is used.");

            kept[binding.Slot] = binding;
        }

        return (new LedMappingFile { Version = CurrentVersion, Bindings = kept.Values.ToList() }, warnings);
    }
}
