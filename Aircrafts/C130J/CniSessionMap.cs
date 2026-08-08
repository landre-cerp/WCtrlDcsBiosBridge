using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Remembers which element of a field is the highlighted one, for as long as the sim keeps
/// issuing that element.
///
/// A toggle is built as one pair of elements per position — lit and plain, same word, same
/// place — and the sim draws one of each. Nothing in the indication says which, and the pair is
/// identified only by a GUID the sim regenerates every session, so the answer cannot be shipped
/// in the schema. Within a session those GUIDs hold steady, which is what makes an answer worth
/// keeping once it has been had.
///
/// Two things give answers, and both are written here:
///
/// <list type="bullet">
/// <item>What the page itself proved this frame — a variant that spells its text differently, a
/// word the module only draws in one state, a radio device answering is_on(). Those readings
/// are about the state, but because the element carrying it is named, they are equally readings
/// about the element, and the element outlives the frame that revealed it. Their reach is what
/// grows: ADF spelling itself "ADF/ " when unselected settles ADF, and settling ADF is what says
/// MN and BTH beside it are not the lit ones.</item>
/// <item>The crew turning a rotary. Turn it and two members swap element, the rest hold. A
/// member that held is unlit on both sides of the turn — had it been the lit one, the others
/// would have had no reason to move. That is one fact per turn and they compose, which is the
/// whole of what was needed to read GPS/LAST/REF off a capture where nothing else moved.</item>
/// </list>
///
/// The two readings compose through one rule: a field draws one of two forms and no more, so
/// knowing either settles the other.
///
/// What it costs: nothing is known about a field until something has given it away, and
/// everything is dropped when the page is rebuilt. Until then the field draws unlit, which is
/// what it did before any of this. It never guesses.
/// </summary>
internal sealed class CniSessionMap
{
    /// <summary>What is known about a field's elements: identity to lit or not.</summary>
    private readonly Dictionary<string, Dictionary<string, bool>> _known = new(StringComparer.Ordinal);

    /// <summary>The elements last seen drawn for each rotary, member by member.</summary>
    private readonly Dictionary<string, Dictionary<string, string>> _seen = new(StringComparer.Ordinal);

    /// <summary>What is drawing each field on the page just observed.</summary>
    private readonly Dictionary<string, string> _current = new(StringComparer.Ordinal);

    /// <summary>
    /// The elements last seen behind each page's fixed text, by slot. A page's literals cannot
    /// move with any state, so they are what tells a rebuild from a switch.
    /// </summary>
    private readonly Dictionary<int, Dictionary<int, string>> _landmarks = new();

    /// <summary>
    /// Whether the field is the lit one on the page last observed, where that has been
    /// established. Asked by field rather than by element because a field is identified by
    /// everything it draws, not by any one of them.
    /// </summary>
    public bool? Lit(string field)
    {
        if (!_current.TryGetValue(field, out var drawn)) return null;

        return Known(field, drawn);
    }

    /// <summary>
    /// Takes in the page just received, together with whatever the caller has already worked out
    /// about it this frame.
    /// </summary>
    public void Observe(CniPage page, IReadOnlyList<CniBlock> blocks, CniSlot?[] matched,
                        IReadOnlyDictionary<string, bool?>? states = null)
    {
        // Before anything is read off these elements, and before anything is written about them.
        if (Rebuilt(page, blocks, matched)) Forget(page);

        var drawn = Drawn(blocks, matched);

        _current.Clear();
        foreach (var (field, identity) in drawn) _current[field] = identity;

        if (states is not null) Record(states, drawn);

        foreach (var selector in page.Selectors)
        {
            var members = Members(selector, drawn);

            // A rotary half on screen says nothing safely: a member that is merely missing looks
            // exactly like one that held.
            if (members.Count != selector.Members.Count) continue;

            if (_seen.TryGetValue(selector.Key, out var before))
            {
                Compare(selector, before, members);

                // The turn just seen can settle the picture it came from as much as the one it
                // arrived at: learning that a member held is what leaves only one candidate for
                // what was lit before. GPS/LAST/REF resolves on exactly that.
                Close(selector, before);
            }

            _seen[selector.Key] = members;

            Close(selector, members);
        }
    }

    /// <summary>
    /// Writes down what the frame proved, as a fact about the elements that carried it.
    ///
    /// This is the whole of what the pass adds over watching rotaries turn. A state read off the
    /// page holds for that frame either way; read as a statement about the element drawing it,
    /// it holds until the sim builds another one — and it reaches the members beside it, which a
    /// reading confined to its own frame never could.
    /// </summary>
    private void Record(IReadOnlyDictionary<string, bool?> states,
                        IReadOnlyDictionary<string, string> drawn)
    {
        foreach (var (field, state) in states)
        {
            if (state is not { } lit) continue;
            if (!drawn.TryGetValue(field, out var identity)) continue;

            Mark(field, identity, lit);
        }
    }

    /// <summary>
    /// What is currently drawing each two-state field.
    ///
    /// Every element the field owns, not just one: a position is usually a word plus the slash
    /// beside it, both under the same controller and both swapped together when the field
    /// changes. Reading one of them would work until the day the two are picked in a different
    /// order, so the field is identified by all of them at once.
    ///
    /// Two-state fields only. A controller that drives one element in one form has no other
    /// form to be told apart from, and the rule the whole map turns on — anything that is not
    /// the lit form is the plain one — would be a straight invention on it.
    /// </summary>
    private static Dictionary<string, string> Drawn(IReadOnlyList<CniBlock> blocks,
                                                    CniSlot?[] matched)
    {
        var parts = new Dictionary<string, List<string>>(StringComparer.Ordinal);

        for (var i = 0; i < matched.Length && i < blocks.Count; i++)
        {
            if (matched[i] is not { FieldHasTwoStates: true, Controller: { } field }) continue;
            if (blocks[i].K is not { Length: > 0 } element) continue;

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

    private static Dictionary<string, string> Members(CniSelector selector,
                                                      IReadOnlyDictionary<string, string> drawn)
    {
        var members = new Dictionary<string, string>(StringComparer.Ordinal);

        foreach (var member in selector.Members)
            if (drawn.TryGetValue(member, out var identity)) members[member] = identity;

        return members;
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
    /// A field draws one of two forms and no more, so knowing either one settles the other:
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

        if (elements.TryGetValue(element, out var was) && was != lit) elements.Clear();

        elements[element] = lit;

        // A pair has one of each, so naming the lit one names the other by elimination — which
        // is how a field resolved on one page carries to the next time it is drawn.
        if (!lit) return;
        foreach (var other in elements.Keys.Where(k => k != element).ToList()) elements[other] = false;
    }

    /// <summary>
    /// Whether the sim has built this page afresh since it was last seen.
    ///
    /// Everything here is keyed on element names that only mean anything while the elements
    /// exist, and the reasoning is not merely incomplete afterwards but wrong: an identity that
    /// is unknown because it is new reads, through the pair rule, as the opposite of one that is
    /// on file. So a rebuild has to be noticed rather than waited out.
    ///
    /// The page's own fixed text is what notices it. A literal is written into the page script
    /// and no state can move it, so the element behind "IDENT" is the same one from frame to
    /// frame until the page is rebuilt and then none of them are. Which of the two happened is
    /// therefore settled by the landmarks the two frames have in common, and it takes only one
    /// of them to agree for the elements to still be the old ones.
    /// </summary>
    private bool Rebuilt(CniPage page, IReadOnlyList<CniBlock> blocks, CniSlot?[] matched)
    {
        var now = new Dictionary<int, string>();

        for (var i = 0; i < matched.Length && i < blocks.Count; i++)
        {
            var slot = matched[i];

            // Fixed text, tied to no state and paired with nothing — anything else can change
            // element for reasons that are not a rebuild, which is the one thing this must not
            // mistake. The title and the scratchpad are excluded by the same rule that makes
            // them useless here: the sim names them, so they never carry a GUID at all.
            if (slot is not { IsStatic: true, Controller: null, NamedRole: 0 }) continue;
            if (slot.Counterpart is not null) continue;
            if (blocks[i].K is not { Length: > 0 } element) continue;

            now[slot.N] = element;
        }

        if (!_landmarks.TryGetValue(page.Id, out var before))
        {
            if (now.Count > 0) _landmarks[page.Id] = now;
            return false;
        }

        var shared = 0;
        var agreed = 0;

        foreach (var (n, element) in now)
        {
            if (!before.TryGetValue(n, out var was)) continue;
            shared++;
            if (was == element) agreed++;
        }

        // Merged rather than replaced: a landmark the page happened not to draw this time is
        // still the one it will draw next time, and dropping it would narrow what the frame
        // after that has to go on.
        foreach (var (n, element) in now) before[n] = element;

        return shared > 0 && agreed == 0;
    }

    /// <summary>Drops everything learned about a page whose elements are no longer the same.</summary>
    private void Forget(CniPage page)
    {
        foreach (var slot in page.Slots)
            if (slot.Controller is { } field) _known.Remove(field);

        foreach (var selector in page.Selectors) _seen.Remove(selector.Key);
    }
}
