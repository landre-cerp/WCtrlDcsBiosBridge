using System.Text.RegularExpressions;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Reads a radio's power out of the device readings and says which page field it settles.
///
/// The CNI shows power as OFF/ON with one word highlighted, and nothing on the wire moves when
/// the crew flips it — the two words are built alike and only the element behind them changes.
/// The radio devices answer <c>is_on()</c> outright, so the state exists; the work is knowing
/// which radio a page is talking about.
///
/// That comes from the page itself. A page tuned to 243.000 prints it, and exactly one device
/// is transmitting there, so the two can be matched without either end naming a device number.
/// Device numbers shift between module versions and an index that is right on one install and
/// wrong on another is the failure the CDNU work already ran into.
/// </summary>
internal static class CniRadios
{
    /// <summary>uhf1_power_on and uhf1_power_off are the two words of the field UHF1 powers.</summary>
    private static readonly Regex PowerField = new(@"^(?<radio>.+)_power_(?<word>on|off)$",
                                                   RegexOptions.Compiled);

    /// <summary>
    /// A frequency as the CNI prints it, megahertz with a fractional part — "1/243.000",
    /// "243.000R", " 2/264.000". An unset field reads "---.---" and matches nothing.
    /// </summary>
    private static readonly Regex Megahertz = new(@"(?<mhz>\d+)\.(?<frac>\d{1,3})",
                                                  RegexOptions.Compiled);

    /// <summary>
    /// Adds what the radios prove to <paramref name="states"/>, for the fields whose radio the
    /// page identifies. Fields left untouched stay unknown and are drawn plain.
    /// </summary>
    public static void Observe(Dictionary<string, bool?> states, CniSlot?[] matched,
                               IReadOnlyList<CniBlock> blocks, IReadOnlyList<RadioState>? radios)
    {
        if (radios is not { Count: > 0 }) return;

        var powered = TunedRadios(matched, blocks, radios);
        if (powered.Count == 0) return;

        foreach (var slot in matched)
        {
            if (slot?.Controller is not { } field) continue;

            var parsed = PowerField.Match(field);
            if (!parsed.Success) continue;
            if (!powered.TryGetValue(parsed.Groups["radio"].Value, out var on)) continue;

            // The ON word is lit when the radio is on and the OFF word when it is not, so one
            // reading settles both halves of the pair.
            var selected = parsed.Groups["word"].Value == "on" ? on : !on;
            Record(states, field, selected);
        }
    }

    /// <summary>
    /// The power of every radio the page names, keyed by the controller prefix that names it.
    ///
    /// A page prints its frequency through a controller of the same family — uhf1_chan beside
    /// uhf1_power_on — so the prefix is what carries across, and no device number is spoken at
    /// either end.
    /// </summary>
    private static Dictionary<string, bool> TunedRadios(CniSlot?[] matched,
                                                        IReadOnlyList<CniBlock> blocks,
                                                        IReadOnlyList<RadioState> radios)
    {
        var found = new Dictionary<string, bool>(StringComparer.Ordinal);

        for (var i = 0; i < matched.Length && i < blocks.Count; i++)
        {
            var named = matched[i]?.Source ?? matched[i]?.Controller;
            if (named is null) continue;

            var prefix = Prefix(named);
            if (prefix is null || found.ContainsKey(prefix)) continue;

            var kilohertz = ParseKilohertz(blocks[i].V);
            if (kilohertz is not { } tuned) continue;

            var radio = radios.FirstOrDefault(r => r.Frequency is { } f && Math.Abs(f - tuned) <= 1);
            if (radio?.On is { } on) found[prefix] = on;
        }

        return found;
    }

    /// <summary>
    /// The family a controller belongs to: everything up to its last segment. uhf1_chan and
    /// uhf1_power_on both give "uhf1", which is the whole point.
    /// </summary>
    private static string? Prefix(string controller)
    {
        var cut = controller.IndexOf('_');
        return cut > 0 ? controller[..cut] : null;
    }

    private static long? ParseKilohertz(string? text)
    {
        if (string.IsNullOrEmpty(text)) return null;

        var match = Megahertz.Match(text);
        if (!match.Success) return null;

        if (!long.TryParse(match.Groups["mhz"].Value, out var mhz)) return null;

        var frac = match.Groups["frac"].Value.PadRight(3, '0');
        if (!long.TryParse(frac, out var khz)) return null;

        return mhz * 1000 + khz;
    }

    private static void Record(Dictionary<string, bool?> states, string field, bool selected)
    {
        if (states.TryGetValue(field, out var seen) && seen != selected)
        {
            states[field] = null;
            return;
        }

        states[field] = selected;
    }
}
