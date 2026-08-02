using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Works out which state each of a page's toggleable fields is in, and swaps every element to
/// the variant that matches.
///
/// The indication carries no highlight, and a field's two variants usually read alike, so most
/// of the time the state cannot be known. Where it can, it is because the sim gave itself away:
/// an element the module only builds in one state has arrived, or the two variants of a field
/// spell their text differently and the matcher already had to choose between them on the text.
/// Either way the answer then applies to every element the field drives, which is how the GUARD
/// word in the IDENT column settles the GUARD toggle three lines below it.
///
/// A field with no evidence is drawn unselected — plain and small, on every member. That is not
/// the same as guessing: the alternative is a highlight on the wrong half of a rotary, and the
/// highlight is the whole of what a rotary says.
/// </summary>
internal static class CniVariants
{
    /// <summary>
    /// Rewrites <paramref name="matched"/> in place, one entry per block.
    /// </summary>
    public static void Apply(CniSlot?[] matched,
                             CniPage? page = null,
                             IReadOnlyList<CniBlock>? blocks = null,
                             IReadOnlyList<RadioState>? radios = null,
                             int? shipSolution = null,
                             CniSessionMap? session = null)
    {
        var states = Observe(matched);

        if (page is not null) ObserveAbsences(states, matched, page);
        if (blocks is not null) CniRadios.Observe(states, matched, blocks, radios);

        if (page is not null && blocks is not null && session is not null)
        {
            session.Observe(page, blocks, matched);
            ObserveSession(states, matched, blocks, session);
        }

        for (var i = 0; i < matched.Length; i++)
        {
            var slot = matched[i];
            if (slot?.Counterpart is not { } other) continue;

            if (slot.Controller is { } field
                && states.TryGetValue(field, out var known)
                && known is { } state)
            {
                matched[i] = state == slot.IsSelected ? slot : other;
                continue;
            }

            if (blocks is not null && i < blocks.Count
                && CniShipSolution.Choose(slot, other, blocks[i].V, shipSolution) is { } corner)
            {
                matched[i] = corner;
                continue;
            }

            // No answer, so take whichever of the pair claims the least. A toggle has no neutral
            // state of its own — the module builds one branch per position and each marks its
            // own word — but the two forms of a single word do, and the unmarked one is it.
            matched[i] = Emphasis(slot) <= Emphasis(other) ? slot : other;
        }
    }

    /// <summary>
    /// How loudly a slot claims to be the selected one.
    ///
    /// The inverted material is the usual mark and outranks everything. Where the module does
    /// not use it, size carries the selection alone: POWER UP has no highlight on its two NAV DB
    /// lines and simply draws the loaded one large. Twenty-two pairs across the pages are like
    /// that, and reading only the highlight left them asserting a state on their size — the
    /// device showed 09SEP09OCT24 as the loaded database while the cockpit had 12AUG10SEP25.
    /// </summary>
    private static int Emphasis(CniSlot slot) => (slot.IsInvert ? 2 : 0) + (slot.IsSmall ? 0 : 1);

    /// <summary>
    /// What the crew has already shown us this session.
    ///
    /// Recorded last of the four so that anything derived from the page itself wins: those hold
    /// on the first frame, where this holds only once a rotary has been turned twice.
    /// </summary>
    private static void ObserveSession(Dictionary<string, bool?> states, CniSlot?[] matched,
                                       IReadOnlyList<CniBlock> blocks, CniSessionMap session)
    {
        for (var i = 0; i < matched.Length && i < blocks.Count; i++)
        {
            if (matched[i]?.Controller is not { } field) continue;
            if (states.ContainsKey(field)) continue;

            if (session.Lit(field) is { } lit) states[field] = lit;
        }
    }

    /// <summary>
    /// What the page proves by what it did <em>not</em> draw.
    ///
    /// An element the module builds for one state only is drawn exactly when that state holds,
    /// so its absence is as good as its presence — and a field with two states has nowhere else
    /// to be. On COMM TUNE U1 the word GUARD leaves the IDENT column when the guard comes off,
    /// which is what puts the toggle below it on OFF rather than on neither.
    ///
    /// Only sound because the matcher cannot mislay one of these: they are literal slots, and a
    /// literal whose text disagrees is forbidden outright rather than merely penalised, so a
    /// block carrying that word has no other slot it could have gone to.
    /// </summary>
    private static void ObserveAbsences(Dictionary<string, bool?> states, CniSlot?[] matched,
                                        CniPage page)
    {
        var drawn = matched.Where(s => s is not null).ToHashSet();

        foreach (var slot in page.Slots)
        {
            if (slot.Controller is not { } field) continue;
            if (!slot.FieldHasTwoStates || slot.Counterpart is not null) continue;
            if (!slot.IsStatic || !slot.IsPlaceable) continue;
            if (drawn.Contains(slot) || states.ContainsKey(field)) continue;

            states[field] = !slot.IsSelected;
        }
    }

    /// <summary>
    /// What the drawn elements prove about each field. Null marks a field whose elements
    /// disagree, which would mean the page was misidentified — no state is better than one of
    /// two contradictory ones.
    /// </summary>
    private static Dictionary<string, bool?> Observe(CniSlot?[] matched)
    {
        var states = new Dictionary<string, bool?>(StringComparer.Ordinal);

        foreach (var slot in matched)
        {
            if (slot?.Controller is not { } field) continue;
            if (!Decisive(slot)) continue;

            if (states.TryGetValue(field, out var seen))
            {
                if (seen != slot.IsSelected) states[field] = null;
            }
            else
            {
                states[field] = slot.IsSelected;
            }
        }

        return states;
    }

    /// <summary>
    /// Whether this element being on screen settles its field.
    ///
    /// Two cases. An element with no counterpart is built for one state only, so the sim
    /// drawing it is the answer. An element whose counterpart spells its text differently was
    /// picked by the matcher on that text, which the matcher never gets wrong: a literal slot
    /// whose text disagrees is forbidden outright. Anything else is a coin toss and says
    /// nothing.
    /// </summary>
    private static bool Decisive(CniSlot slot)
    {
        // A controller that never names a state drives one element in one form, so it proves
        // nothing by turning up. MODE S sat on ON for a whole flight on exactly that mistake,
        // its OUT branch having gone unlabelled and left the ON looking like the only one.
        if (!slot.FieldHasTwoStates) return false;

        if (slot.Counterpart is not { } other) return true;
        if (!slot.IsStatic || !other.IsStatic) return false;

        return !string.Equals(slot.Value, other.Value, StringComparison.Ordinal);
    }
}
