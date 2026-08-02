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

    /// <summary>Skipping a schema slot: routine, that is a variant the sim did not draw.</summary>
    private const int GapSlot = -1;

    /// <summary>Leaving a block unplaced: the schema is wrong or the page was misidentified.</summary>
    private const int GapBlock = -8;

    /// <summary>Never pair these. Large enough to dominate, small enough not to overflow.</summary>
    private const int Forbidden = -1_000_000;

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
    /// Returns, for each block, the slot it was matched to, or null where none fitted.
    /// </summary>
    public static CniSlot?[] Align(IReadOnlyList<CniBlock> blocks, IReadOnlyList<CniSlot> slots)
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
            score[i, 0] = score[i - 1, 0] + GapBlock;
            from[i, 0] = 2;
        }

        for (var i = 1; i <= n; i++)
        {
            for (var j = 1; j <= m; j++)
            {
                var pair = Pair(blocks[i - 1], slots[j - 1]);
                var diagonal = pair == Forbidden ? Forbidden : score[i - 1, j - 1] + pair;
                var skipSlot = score[i, j - 1] + GapSlot;
                var skipBlock = score[i - 1, j] + GapBlock;

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

        // A slot with a format is worth more when the text fits it, in proportion to how much
        // the format narrows things down — that is what puts a frequency on the radio's line
        // instead of the identifier's sub-line, where both slots would otherwise score alike
        // and the earlier one would win by default.
        if (slot.Formats is { Count: > 0 } && !slot.FormatAccepts(value))
            return ScoreFormatMiss;

        // The route discontinuity marker. Its element is built like any other - a bare "%s" in
        // legs_progress.lua - so nothing in the schema says it holds one thing only, and it sits
        // between the waypoint name and the via that a leg always draws. Three consecutive
        // slots for two blocks, the middle one optional, scores identically whichever is left
        // empty, so every leg took the marker's slot for its via and the line came out centred
        // instead of against the left margin. Reading the banner is the one thing that settles
        // it, and the FMS writes no other text there.
        if (slot.Source is { } source && source.EndsWith("_disc", StringComparison.Ordinal))
            return value.Contains("DISCONTINUIT", StringComparison.Ordinal) ? ScoreLiteralHit : Forbidden;

        if (slot.FormatWeight > 0)
            return ScoreDynamic + slot.FormatWeight;

        // A slot built from a bare "%s" accepts anything and so says nothing about the block
        // sitting in it. Rewarding that was what made LEGS drift: a leg is ten slots of which
        // the sim draws seven, four of them accepting anything, and the optional discontinuity
        // is one. Worth +1 against a -1 skip, it swallowed the next field on the line and every
        // leg below inherited the shift. Scored under the skip it gives way instead - and a
        // block still lands in it rather than being dropped, which costs -8.
        return ScoreUninformative;
    }
}
