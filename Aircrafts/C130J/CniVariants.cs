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
/// Every such answer also goes to <see cref="CniSessionMap"/>, which keeps it against the element
/// that gave it rather than against the frame. That is what carries a reading past the moment it
/// was available — the radio that settled PWR while its frequency was on the page still settles
/// it on the page that does not print one — and past the field it was about, since a rotary with
/// one member accounted for has said something about the others.
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
            // Handed everything the three passes above settled, because each of those is as much
            // a reading of the element that carried it as of the state — and the element is
            // still there on the frames where the reading is not.
            session.Observe(page, blocks, matched, states);
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

            matched[i] = Neutral(slot, other);
        }
    }

    /// <summary>
    /// Which of a pair to draw when nothing has settled the field.
    ///
    /// Two shapes, and they want opposite answers.
    ///
    /// A toggle marks its selection with the inverted material, and it has no neutral state of
    /// its own: the module builds one branch per position and each marks its own word, so
    /// picking either branch asserts a position. What does have a neutral state is a single
    /// word taken on its own — the two forms of the ON in COMM TUNE's PWR are the same word
    /// marked and unmarked — so each is drawn in whichever of its forms claims the least, and
    /// the toggle comes out plain on both words.
    ///
    /// Where the module marks the selection with size alone, that reasoning has nothing to
    /// stand on: with no mark to drop, the smaller of the two forms is not the quieter one but
    /// simply the other state. These are the twenty-two soft-key labels and status lines built
    /// large and redrawn small once their action is spent or their state is set — MSTR AV ON,
    /// AUTONAV, START, STOP — and drawing them small was saying the crew had already used them.
    /// The unselected form is what the page shows before anything has happened, so that is the
    /// one drawn, whichever of the two sizes it happens to be.
    /// </summary>
    private static CniSlot Neutral(CniSlot slot, CniSlot other)
    {
        if (!slot.IsInvert && !other.IsInvert && slot.IsSelected != other.IsSelected)
            return slot.IsSelected ? other : slot;

        return Emphasis(slot) <= Emphasis(other) ? slot : other;
    }

    /// <summary>
    /// How loudly a slot claims to be the selected one.
    ///
    /// The inverted material is the usual mark and outranks everything, and size comes after it
    /// because the two travel together: the selected member of a field is drawn large as well as
    /// highlighted, so reading only the highlight left the pairs that use both asserting a state
    /// on their size.
    /// </summary>
    private static int Emphasis(CniSlot slot) => (slot.IsInvert ? 2 : 0) + (slot.IsSmall ? 0 : 1);

    /// <summary>
    /// What the elements on screen have already been shown to be, earlier in the session.
    ///
    /// Read last of the four, and only for the fields the other three left open. It is never in
    /// conflict with them — what it holds was put there by them — but it is the older reading of
    /// the two, and a field the page is settling for itself this frame has no need of it.
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

        // A table column, where the matcher pays to seat a block on the highlighted half and so
        // only does it where every plain reading came out strictly worse. That makes the seating
        // itself an answer, and it is the only one LANDING DATA's header row gives: "0", "50"
        // and "100" read the same in both halves, but the sim sends the selected one first, and
        // no plain reading can put "100" ahead of the two that follow it.
        //
        // Columns only. A toggle offers the matcher two branches holding one highlighted word
        // each, so the two cost the same and it takes the earlier by tie-break rather than on
        // any evidence — reading that as proof lit SQL and TONE on COMM TUNE U1, which is
        // exactly the invention this whole pass exists to avoid.
        if (slot.Column is not null && slot.IsInvert && !other.IsInvert) return true;

        if (!slot.IsStatic || !other.IsStatic) return false;

        return !string.Equals(slot.Value, other.Value, StringComparison.Ordinal);
    }
}
