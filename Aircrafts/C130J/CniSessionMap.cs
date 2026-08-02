using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Learns which of a rotary member's two elements is the lit one, by watching the crew turn it.
///
/// A rotary is built as one pair of elements per position — lit and plain, same word, same
/// place — and the sim draws one of each. Nothing in the indication says which, and the pair is
/// identified only by a GUID the sim regenerates every session, so the answer cannot be shipped
/// in the schema. It can be worked out in flight, from a single fact: exactly one member is lit.
///
/// Turn the rotary and two members swap element, the rest hold. A member that held is unlit on
/// both sides of the turn — had it been the lit one, the others would have had no reason to
/// move. That is one fact per turn, and they compose: once every member but one is known unlit,
/// the last is lit; and once one element of a pair is known lit, the other is not. Two turns of
/// a three-position rotary settle it completely, which is the whole of what was needed to read
/// GPS/LAST/REF off a capture.
///
/// What it costs: nothing is known until the crew has actually turned the thing, and everything
/// is forgotten when the elements are rebuilt. Until then the field draws unlit, which is what
/// it did before any of this. It never guesses.
/// </summary>
internal sealed class CniSessionMap
{
    /// <summary>What is known about a field's elements: GUID to lit or not.</summary>
    private readonly Dictionary<string, Dictionary<string, bool>> _known = new(StringComparer.Ordinal);

    /// <summary>The elements last seen drawn for each rotary, member by member.</summary>
    private readonly Dictionary<string, Dictionary<string, string>> _seen = new(StringComparer.Ordinal);

    /// <summary>What is drawing each member on the page just observed.</summary>
    private readonly Dictionary<string, string> _current = new(StringComparer.Ordinal);

    /// <summary>
    /// Whether the field is the lit one on the page last observed, where that has been
    /// established. Asked by field rather than by element because a member is identified by
    /// everything it draws, not by any one of them.
    /// </summary>
    public bool? Lit(string field)
    {
        if (!_current.TryGetValue(field, out var drawn)) return null;

        return Known(field, drawn);
    }

    public void Observe(CniPage page, IReadOnlyList<CniBlock> blocks, CniSlot?[] matched)
    {
        _current.Clear();

        foreach (var selector in page.Selectors)
        {
            var drawn = Drawn(selector, blocks, matched);

            // A rotary half on screen says nothing safely: a member that is merely missing looks
            // exactly like one that held.
            if (drawn.Count != selector.Members.Count) continue;

            if (_seen.TryGetValue(selector.Key, out var before))
            {
                Compare(selector, before, drawn);

                // The turn just seen can settle the picture it came from as much as the one it
                // arrived at: learning that a member held is what leaves only one candidate for
                // what was lit before. GPS/LAST/REF resolves on exactly that.
                Close(selector, before);
            }

            _seen[selector.Key] = drawn;

            Close(selector, drawn);

            foreach (var (member, identity) in drawn) _current[member] = identity;
        }
    }

    /// <summary>
    /// What is currently drawing each member of the rotary.
    ///
    /// Every element the member owns, not just one: a position is usually a word plus the slash
    /// beside it, both under the same controller and both swapped together when the rotary
    /// turns. Reading one of them would work until the day the two are picked in a different
    /// order, so the member is identified by all of them at once.
    /// </summary>
    private static Dictionary<string, string> Drawn(CniSelector selector,
                                                    IReadOnlyList<CniBlock> blocks,
                                                    CniSlot?[] matched)
    {
        var parts = new Dictionary<string, List<string>>(StringComparer.Ordinal);

        for (var i = 0; i < matched.Length && i < blocks.Count; i++)
        {
            if (matched[i]?.Controller is not { } field) continue;
            if (blocks[i].K is not { Length: > 0 } element) continue;
            if (!selector.Members.Contains(field)) continue;

            if (!parts.TryGetValue(field, out var elements)) parts[field] = elements = new List<string>();
            elements.Add(element);
        }

        var drawn = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (var (field, elements) in parts)
        {
            elements.Sort(StringComparer.Ordinal);
            drawn[field] = string.Join('|', elements);
        }

        return drawn;
    }

    private void Compare(CniSelector selector,
                         IReadOnlyDictionary<string, string> before,
                         IReadOnlyDictionary<string, string> now)
    {
        var held = selector.Members.Where(m => before.TryGetValue(m, out var was) && was == now[m]).ToList();

        // Nothing moved, so nothing happened. Every member moved, so the page was rebuilt and
        // the old GUIDs mean nothing any more — including whatever was learned about them.
        if (held.Count == selector.Members.Count) return;
        if (held.Count == 0)
        {
            foreach (var member in selector.Members) _known.Remove(member);
            return;
        }

        foreach (var member in held) Mark(member, now[member], lit: false);
    }

    /// <summary>
    /// Reads off what the current picture forces, given everything known so far.
    /// </summary>
    private void Close(CniSelector selector, IReadOnlyDictionary<string, string> drawn)
    {
        var lit = selector.Members.Where(m => Known(m, drawn[m]) == true).ToList();

        if (lit.Count > 0)
        {
            // The lit one is spoken for, so the rest are not.
            foreach (var member in selector.Members.Except(lit)) Mark(member, drawn[member], lit: false);
            return;
        }

        var open = selector.Members.Where(m => Known(m, drawn[m]) is null).ToList();
        if (open.Count == 1) Mark(open[0], drawn[open[0]], lit: true);
    }

    /// <summary>
    /// Whether that identity is the lit one, reading through what the pair implies.
    ///
    /// A member draws one of two forms and no more, so knowing either one settles the other:
    /// anything that is not the form known to be lit is unlit, and anything that is not the form
    /// known to be unlit is lit. Without that second reading the map needs a third turn to say
    /// what two have already established.
    /// </summary>
    private bool? Known(string field, string identity)
    {
        if (!_known.TryGetValue(field, out var elements)) return null;
        if (elements.TryGetValue(identity, out var lit)) return lit;

        if (elements.ContainsValue(true)) return false;
        if (elements.ContainsValue(false)) return true;
        return null;
    }

    private void Mark(string field, string element, bool lit)
    {
        if (!_known.TryGetValue(field, out var elements))
            _known[field] = elements = new Dictionary<string, bool>(StringComparer.Ordinal);

        elements[element] = lit;

        // A pair has one of each, so naming the lit one names the other by elimination — which
        // is how a member resolved on one page carries to the next time it is drawn.
        if (!lit) return;
        foreach (var other in elements.Keys.Where(k => k != element).ToList()) elements[other] = false;
    }
}
