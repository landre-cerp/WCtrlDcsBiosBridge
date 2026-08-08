using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Pairs the blocks the sim emitted with the slots the page schema describes, so each block
/// can be given the position the indication does not carry.
///
/// The schema always holds more slots than arrive: it lists every variant of every field,
/// while the sim sends only what is visible. Which variants were dropped depends on aircraft
/// state and they are not always at the end — on POWER UP a single placeholder is replaced
/// mid-page by two elements, shifting everything below it. So ordinals cannot be used
/// directly, and this aligns the two sequences instead.
///
/// Both are in Add() order, which makes it an alignment problem rather than a search: a
/// Needleman-Wunsch pass over ~60x70 cells, scoring a block against a slot by whether the
/// literal text agrees. Static values act as landmarks that pin the alignment; dynamic fields
/// take whatever position falls between them.
/// </summary>
internal static class CniBlockMatcher
{
    private const int ScoreLiteralHit = 4;
    private const int ScoreContainer = 3;
    private const int ScoreDynamic = 1;

    /// <summary>
    /// A slot that would accept any text at all. Below <see cref="GapSlot"/> so the alignment
    /// passes over it rather than filling it for the sake of filling it, and well above
    /// <see cref="GapBlock"/> so a block with nowhere better still lands there.
    /// </summary>
    private const int ScoreUninformative = -2;

    /// <summary>
    /// The text does not fit the slot's format. Allowed, because the sim can format a field in
    /// ways the page script does not spell out, but never preferred over a slot it does fit.
    /// </summary>
    private const int ScoreFormatMiss = -2;

    /// <summary>
    /// An empty field on a slot whose format carries punctuation the text does not. Below
    /// <see cref="ScoreUninformative"/> so a plain "%s" slot takes it in preference, and above
    /// <see cref="GapBlock"/> so it still lands here rather than nowhere.
    /// </summary>
    private const int ScoreUnsetMismatch = -3;

    /// <summary>Skipping a schema slot: routine, that is a variant the sim did not draw.</summary>
    private const int GapSlot = -1;

    /// <summary>Leaving a block unplaced: the schema is wrong or the page was misidentified.</summary>
    private const int GapBlock = -8;

    /// <summary>
    /// Leaving a blank unplaced, which costs the screen nothing: the sim emits an element for
    /// every field of every leg whether or not it holds anything, and a blank that finds no slot
    /// would have drawn nothing had it found one.
    ///
    /// Priced apart because the flat cost had the alignment giving up a real value to keep a
    /// blank. The last leg of a broken route sends one element more than the page has room for,
    /// and what went overboard was the landing altitude rather than one of the six blanks
    /// beside it.
    /// </summary>
    private const int GapEmptyBlock = -4;

    /// <summary>
    /// The route discontinuity banner on its own slot. Worth far more than any other landmark
    /// because placing it is what a whole leg's layout hangs on, and because a leg the route is
    /// broken at draws two fields out of eight — so seating the banner means passing over six
    /// slots, which at a gap apiece already costs as much as giving the block up. Scored at a
    /// literal's worth, the alignment took that trade and dropped the banner, and the leg's own
    /// name went to the bearing beside it with every leg below inheriting the shift.
    /// </summary>
    private const int ScoreBanner = 14;

    /// <summary>Never pair these. Large enough to dominate, small enough not to overflow.</summary>
    private const int Forbidden = -1_000_000;

    /// <summary>
    /// What it costs to seat a block on the highlighted half of a pair.
    ///
    /// Small, and only ever applied where the schema offers both halves, so it decides nothing
    /// but a tie — which is exactly the case that was going wrong. A table row is built as every
    /// highlighted cell followed by every plain one, and the two halves of a cell accept the same
    /// text, so the alignment scored them alike and the index tie-break took the earliest pair it
    /// could reach: the highlighted cell and then the plain twin of that same cell, both at the
    /// same place, the second drawn over the first. LANDING DATA lost a whole column of approach
    /// speeds that way, and TOLD INDEX three of its six lines.
    ///
    /// Preferring the plain half unpicks that, because a row draws at most one highlighted cell
    /// and the sim sends the rest plain. Where the highlighted half is the only reading that
    /// fits — the FLAPS header sends "100" before "0", which nothing but the highlighted 100 can
    /// account for — a unit of tie-break is nowhere near enough to give a block up at -8.
    /// </summary>
    private const int PenaltyHighlighted = -1;

    /// <summary>
    /// Blocks in document order, children immediately after their container, matching the
    /// order the schema records and the <c>n</c> the export assigns.
    /// </summary>
    public static List<CniBlock> Flatten(IEnumerable<CniBlock>? blocks)
    {
        var flat = new List<CniBlock>();
        Walk(blocks, flat);
        return flat;

        static void Walk(IEnumerable<CniBlock>? nodes, List<CniBlock> into)
        {
            if (nodes == null) return;
            foreach (var node in nodes)
            {
                into.Add(node);
                Walk(node.C, into);
            }
        }
    }

    /// <summary>
    /// Returns, for each block, the slot it was matched to.
    ///
    /// Two passes, because one column of a table settles the rest of it. The first pass prefers
    /// the plain half of every pair, so a highlighted slot is only ever seated where nothing
    /// else fits — the FLAPS header sending "100" before "0" and "50", which no plain reading
    /// can account for. That names the selected column. The second pass then stops charging for
    /// the highlighted half in that column alone, which is what lets the three rows of speeds
    /// beneath the header put their first value in the column the header pointed at instead of
    /// the leftmost one.
    ///
    /// One extra pass only, and only where the first found something: a column the page never
    /// settles is aligned exactly as before.
    /// </summary>
    public static CniSlot?[] Align(IReadOnlyList<CniBlock> blocks, IReadOnlyList<CniSlot> slots)
    {
        var first = Align(blocks, slots, null, null);

        var selected = first
            .Where(s => s is { IsInvert: true, Column: not null })
            .Select(s => s!.Column!)
            .ToHashSet(StringComparer.Ordinal);

        // A leg the route is broken at draws two things: the banner, and its own name on the
        // line below. Everything else that leg owns sits on the banner's line, so the line is
        // the banner's alone. Without that the leg's name — a run of dashes, like any empty
        // field — went to the distance beside the banner instead, which reads the same and
        // comes first, and every leg below inherited the shift.
        var banners = first
            .Where(IsMarker)
            .Select(s => s!.Line!.Value)
            .ToHashSet();

        return selected.Count == 0 && banners.Count == 0
            ? first
            : Align(blocks, slots, selected, banners);
    }

    /// <summary>What it costs to leave this block unplaced.</summary>
    private static int Gap(CniBlock block) =>
        string.IsNullOrEmpty(block.V) ? GapEmptyBlock : GapBlock;

    private static bool IsMarker(CniSlot? slot) =>
        slot is { Line: not null, Source: { } source }
        && source.EndsWith("_disc", StringComparison.Ordinal);

    /// <summary>The waypoint name, the one field a broken leg still draws.</summary>
    private static bool IsLegName(CniSlot slot) =>
        slot.Source is { } source && source.EndsWith("_name", StringComparison.Ordinal);

    /// <summary>
    /// Whether the text is nothing but the module's own way of writing an empty field: dashes,
    /// the box character the export sends as ']', and the spaces between them.
    /// </summary>
    private static bool Unset(string text)
    {
        if (text.Length == 0) return false;

        foreach (var ch in text)
        {
            if (ch is not ('-' or ']' or ' ')) return false;
        }
        return true;
    }

    private static CniSlot?[] Align(IReadOnlyList<CniBlock> blocks, IReadOnlyList<CniSlot> slots,
                                    IReadOnlySet<string>? selected, IReadOnlySet<int>? banners)
    {
        var result = new CniSlot?[blocks.Count];
        if (blocks.Count == 0 || slots.Count == 0) return result;

        slots = OnePerCell(slots);
        int n = blocks.Count, m = slots.Count;

        // score[i, j] is the best alignment of the first i blocks against the first j slots.
        // Ties are common — a page usually ends on blank fields that fit any number of
        // remaining slots equally well — so a second figure breaks them: the total of the slot
        // indices used. Preferring the smaller keeps a value on the slot the sim reached
        // first, which is where COMM TUNE INDEX puts a frequency on the radio's own line
        // rather than the sub-line beneath it.
        var score = new int[n + 1, m + 1];
        var used = new int[n + 1, m + 1];
        var from = new byte[n + 1, m + 1];   // 0 diagonal, 1 skip slot, 2 skip block

        for (var j = 1; j <= m; j++)
        {
            score[0, j] = score[0, j - 1] + GapSlot;
            from[0, j] = 1;
        }
        for (var i = 1; i <= n; i++)
        {
            score[i, 0] = score[i - 1, 0] + Gap(blocks[i - 1]);
            from[i, 0] = 2;
        }

        for (var i = 1; i <= n; i++)
        {
            for (var j = 1; j <= m; j++)
            {
                var pair = Score(blocks[i - 1], slots[j - 1], selected, banners);
                var diagonal = pair == Forbidden ? Forbidden : score[i - 1, j - 1] + pair;
                var skipSlot = score[i, j - 1] + GapSlot;
                var skipBlock = score[i - 1, j] + Gap(blocks[i - 1]);

                var best = Math.Max(diagonal, Math.Max(skipSlot, skipBlock));

                var diagonalUsed = used[i - 1, j - 1] + j;
                var skipSlotUsed = used[i, j - 1];
                var skipBlockUsed = used[i - 1, j];

                if (diagonal == best &&
                    (skipSlot != best || diagonalUsed <= skipSlotUsed) &&
                    (skipBlock != best || diagonalUsed <= skipBlockUsed))
                {
                    score[i, j] = diagonal;
                    used[i, j] = diagonalUsed;
                    from[i, j] = 0;
                }
                else if (skipSlot == best && (skipBlock != best || skipSlotUsed <= skipBlockUsed))
                {
                    score[i, j] = skipSlot;
                    used[i, j] = skipSlotUsed;
                    from[i, j] = 1;
                }
                else
                {
                    score[i, j] = skipBlock;
                    used[i, j] = skipBlockUsed;
                    from[i, j] = 2;
                }
            }
        }

        var x = n;
        var y = m;
        while (x > 0 && y > 0)
        {
            switch (from[x, y])
            {
                case 0:
                    result[x - 1] = slots[y - 1];
                    x--; y--;
                    break;
                case 1:
                    y--;
                    break;
                default:
                    x--;
                    break;
            }
        }

        return result;
    }

    /// <summary>
    /// Drops the second of any two dynamic slots that are the same cell drawn at two sizes.
    ///
    /// A leg's speed exists twice at line 2 column 19, once small and once large, and the
    /// module draws whichever the state calls for - never both. The alignment had no way to
    /// know that: both accept the same text, so filling the pair was worth twice filling one,
    /// and a leg helped itself to the next leg's first two fields to do it. That is what made
    /// the values walk down the page a slot at a time.
    ///
    /// The rule is narrow on purpose, because sharing a cell does not by itself mean the two
    /// are exclusive: COMM TUNE INDEX draws the highlighted GUARD over the identifier it
    /// replaces, and both blocks arrive. What marks a size pair is the module's own naming -
    /// legs_speed against legs_speed_ord, the ordinary field and the one it draws large.
    /// Which of the two sizes to use is settled by CniVariants, on the slot that survives here.
    /// </summary>
    private static IReadOnlyList<CniSlot> OnePerCell(IReadOnlyList<CniSlot> slots)
    {
        List<CniSlot>? kept = null;

        for (var i = 0; i < slots.Count; i++)
        {
            var slot = slots[i];
            var duplicate = i > 0 && SameCell(slots[i - 1], slot);

            if (duplicate && kept is null)
            {
                kept = new List<CniSlot>(slots.Count);
                for (var j = 0; j < i; j++) kept.Add(slots[j]);
            }

            if (!duplicate) kept?.Add(slot);
        }

        return kept ?? slots;
    }

    private static bool SameCell(CniSlot a, CniSlot b) =>
        a.Line is not null
        && a.Line == b.Line && a.Col == b.Col && a.Anchor == b.Anchor
        && a.Value is null && b.Value is null
        && a.Source is { } source && b.Source == source + "_ord";

    /// <summary>
    /// What it is worth to pair this block with this slot.
    ///
    /// A literal slot whose text disagrees is forbidden outright rather than merely penalised:
    /// those landmarks are what hold the alignment together, and letting one slip lets the
    /// whole page drift by a slot.
    /// </summary>
    private static int Score(CniBlock block, CniSlot slot, IReadOnlySet<string>? selected,
                             IReadOnlySet<int>? banners)
    {
        var fit = Pair(block, slot);

        if (fit == Forbidden) return fit;

        // A leg the route is broken at has no bearing, no distance, no time, no speed and no
        // altitude — the FMS has nothing to compute them from. It draws the banner and the
        // leg's name, across the two lines the leg owns, and that is all. Every other field of
        // those two lines is therefore not on screen, and a block that lands in one is a block
        // stolen from the leg below.
        //
        // Static text is left alone: a page writing a literal there is not a leg at all.
        if (banners is not null && slot.Line is { } line && !slot.IsStatic
            && (banners.Contains(line) || banners.Contains(line - 1))
            && !IsMarker(slot) && !IsLegName(slot))
        {
            return Forbidden;
        }
        if (slot.Counterpart is not { } other) return fit;

        var inSelectedColumn = slot.Column is { } column
                               && selected is not null
                               && selected.Contains(column);

        // A cell in the selected column is drawn highlighted, so its plain half is not drawn at
        // all and no block can belong to it. Saying so is what stops the two halves being filled
        // at once: with flap 50 selected the approach speeds arrive "135 153 126", and 135 took
        // the highlighted half of column 50 while 126 took the plain half of the same column,
        // one on top of the other, leaving column 100 empty. The tie-break cannot see that — the
        // two halves are separated by three slots and score alike — so it has to be ruled out
        // rather than out-scored.
        if (inSelectedColumn) return slot.IsInvert || !other.IsInvert ? fit : Forbidden;

        // Only where the schema really does offer both halves. An element the module builds
        // highlighted and nothing else — the scratchpad's error line, the page counters — has no
        // plain twin to be preferred over, and docking it would only push its block somewhere
        // worse.
        return slot.IsInvert ? fit + PenaltyHighlighted : fit;
    }

    private static int Pair(CniBlock block, CniSlot slot)
    {
        var value = block.V ?? "";
        var hasChildren = block.C is { Count: > 0 };

        // The title and the scratchpad are the only elements the sim names rather than giving
        // a fresh GUID, so for those two there is nothing to infer. Pinning them also stops
        // them being traded against a neighbour: the scratchpad sits at the left of the bottom
        // line and an error message sits at the right, and they read very differently.
        var role = block.K switch
        {
            "cni_title" => 1,
            "cni_scratchpad" => 2,
            _ => 0,
        };

        if (role != 0 || slot.NamedRole != 0)
            return role == slot.NamedRole ? ScoreLiteralHit : Forbidden;

        if (slot.IsContainer)
        {
            if (hasChildren) return ScoreContainer;
            // A toggle whose words are all hidden still draws its container, empty.
            return value.Length == 0 ? ScoreDynamic : Forbidden;
        }

        if (hasChildren) return Forbidden;

        if (slot.IsStatic)
            return value == slot.Value ? ScoreLiteralHit : Forbidden;

        // The route discontinuity marker, settled before anything else is asked. Its element is
        // built like any other — a bare "%s" in legs_progress.lua — so nothing in the schema
        // says it holds one thing only, and it sits among the fields a leg always draws.
        // Reading the banner is what settles it, in both directions: the marker's slot takes
        // nothing else, and the banner goes nowhere else. The FMS writes no other text there.
        //
        // Ahead of the format test on purpose. A slot whose format the banner cannot fit still
        // scores -2 for missing it, which is enough to seat it on a bearing when the leg it
        // belongs to has been passed over — and then the leg's own name has nowhere left to go.
        var banner = value.Contains("DISCONTINUIT", StringComparison.Ordinal);
        var marker = slot.Source is { } source && source.EndsWith("_disc", StringComparison.Ordinal);

        if (banner || marker) return banner && marker ? ScoreBanner : Forbidden;

        // A slot with a format is worth more when the text fits it, in proportion to how much
        // the format narrows things down — that is what puts a frequency on the radio's line
        // instead of the identifier's sub-line, where both slots would otherwise score alike
        // and the earlier one would win by default.
        if (slot.Formats is { Count: > 0 } && !slot.FormatAccepts(value))
            return ScoreFormatMiss;

        // A field with nothing in it is drawn as a run of dashes whatever kind of field it is,
        // so a format full of punctuation says nothing about it and is evidence against: the
        // sim would have written that punctuation had the field been the one this slot names.
        // The dashes a broken route writes for a leg's name were landing on the bearing beside
        // it — "%03.f^" and a bare "%s" both take them, and the bearing is the earlier slot —
        // and every leg below inherited the shift. The placeholders that keep their punctuation
        // are unaffected: "----:--" is still an hour and "---/" still a speed.
        if (slot.FormatWeight > 0)
            return Unset(value) ? ScoreUnsetMismatch : ScoreDynamic + slot.FormatWeight;

        // A slot built from a bare "%s" accepts anything and so says nothing about the block
        // sitting in it. Rewarding that was what made LEGS drift: a leg is ten slots of which
        // the sim draws seven, four of them accepting anything, and the optional discontinuity
        // is one. Worth +1 against a -1 skip, it swallowed the next field on the line and every
        // leg below inherited the shift. Scored under the skip it gives way instead - and a
        // block still lands in it rather than being dropped, which costs -8.
        return ScoreUninformative;
    }
}
