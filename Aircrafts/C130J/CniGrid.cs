using System.Text;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>A run of same-styled text on one row, ready to hand to the compositor.</summary>
internal sealed record CniRun(int Line, int Column, string Text, bool Small, bool Invert);

/// <summary>
/// Lays a CNI page onto a 25x14 grid.
///
/// Both mappings are now exact. The CNI has thirteen lines plus a scratchpad against the
/// panel's fourteen rows, and it is 25 characters across — which the panel will run, since
/// the grid is declared to it rather than fixed in the hardware. Columns therefore arrive
/// from the schema as the module itself counted them.
/// </summary>
internal static class CniGrid
{
    public const int Columns = 25;
    public const int Lines = 14;

    /// <summary>
    /// The CNI font repurposes two ASCII slots. Established from captures rather than the
    /// font tables: '^' appears inside coordinates ("N42^14.56") and ']' fills unset fields
    /// ("]]]KT", "]]]]]FT"). Unlike the F-14's CDNU there are no control codes at all.
    ///
    /// The empty field goes to U+2610 and not to U+25A1, which is the squarer of the two and
    /// the obvious choice on paper. The CDU only draws what its font defines, the C-130J runs
    /// on the A-10C's, and that font carries U+2610 in both sizes and U+25A1 in neither — so
    /// the placeholders were being sent and silently dropped.
    /// </summary>
    private static readonly Dictionary<char, char> Glyphs = new()
    {
        ['^'] = '°',   // degree
        [']'] = '☐',   // empty entry field
    };

    /// <summary>
    /// Text size is carried through, because on the CNI it means something: it marks what the
    /// crew can edit, and the selected member of a group is drawn large.
    ///
    /// Size and highlight travel together, because on the module they are one decision: the
    /// selected member of a field is the one drawn large and inverted. Which member that is has
    /// already been settled by <see cref="CniVariants"/>, so both are simply read off the slot
    /// here.
    /// </summary>
    public static IReadOnlyList<CniRun> Render(CniData? data, CniPage? page,
                                              CniSessionMap? session = null)
    {
        var grid = new char[Lines, Columns];
        var small = new bool[Lines, Columns];
        var invert = new bool[Lines, Columns];
        for (var l = 0; l < Lines; l++)
            for (var c = 0; c < Columns; c++)
                grid[l, c] = ' ';

        // Which slot's anchor each cell came from. Two elements sharing an anchor are the sim
        // drawing one over the other, which must be honoured; two with different anchors are
        // meant to sit side by side, and that is where reflow belongs.
        var origin = new int[Lines, Columns];
        for (var l = 0; l < Lines; l++)
            for (var c = 0; c < Columns; c++)
                origin[l, c] = int.MinValue;

        if (data != null && page != null)
        {
            var blocks = CniBlockMatcher.Flatten(data.Blocks);
            var matched = CniBlockMatcher.Align(blocks, page.Slots);
            CniVariants.Apply(matched, page, blocks, data.Radios, data.ShipSolution, session);

            for (var i = 0; i < blocks.Count; i++)
            {
                var slot = matched[i];
                if (slot is null || !slot.IsPlaceable) continue;

                var text = MapGlyphs(blocks[i].V);
                if (text.Length == 0) continue;

                Place(grid, small, invert, origin, slot, text);
            }
        }

        return ToRuns(grid, small, invert);
    }

    /// <summary>Rows as plain strings. For tests and diagnostics, not for the device.</summary>
    public static string[] ToLines(IReadOnlyList<CniRun> runs)
    {
        var rows = new char[Lines][];
        for (var l = 0; l < Lines; l++)
        {
            rows[l] = new char[Columns];
            Array.Fill(rows[l], ' ');
        }

        foreach (var run in runs)
            for (var i = 0; i < run.Text.Length && run.Column + i < Columns; i++)
                rows[run.Line][run.Column + i] = run.Text[i];

        return rows.Select(r => new string(r)).ToArray();
    }

    /// <summary>
    /// Where a run of <paramref name="length"/> characters begins, given its anchor and the
    /// column it is anchored on. Shared with the tests so that what they check is the same
    /// arithmetic the renderer uses, not a second copy of it.
    /// </summary>
    public static int StartColumn(CniAnchor anchor, int origin, int length) => anchor switch
    {
        CniAnchor.Right => origin - length,
        CniAnchor.Center => origin - length / 2,
        _ => origin,
    };

    private static void Place(char[,] grid, bool[,] small, bool[,] invert, int[,] origins,
                              CniSlot slot, string text)
    {
        var line = slot.Line!.Value;
        if (line < 0 || line >= Lines) return;

        // Schema columns are the module's own: 0 at the left edge, 25 at the right.
        var origin = slot.Col!.Value;

        var start = StartColumn(slot.Anchor, origin, text.Length);
        start = Reflow(grid, origins, line, start, text.Length, slot.Anchor, origin);

        for (var i = 0; i < text.Length; i++)
        {
            var col = start + i;
            if (col < 0 || col >= Columns) continue;

            grid[line, col] = text[i];
            small[line, col] = slot.IsSmall;
            invert[line, col] = slot.IsInvert;
            origins[line, col] = origin;
        }
    }

    /// <summary>
    /// Nudges a run clear of text already on the line.
    ///
    /// The CNI is not a fixed grid. A page can put several fonts on one line, and the module
    /// positions each label at the running width of the ones before it — so a seven-character
    /// label set in the narrow font takes about six large-character widths. On a monospaced
    /// display those seven characters need seven cells, and the label that follows lands on
    /// top of them: "ROUTE 1" and "ROUTE 2" collide over the "/" between them.
    ///
    /// Absolute columns cannot express that, so where a collision would occur the run gives
    /// way instead. It only ever triggers on the mixed-font lines; everywhere else the
    /// computed position is already free. On/off variants never collide because the sim sends
    /// only one of them.
    /// </summary>
    private static int Reflow(char[,] grid, int[,] origins, int line, int start, int length,
                              CniAnchor anchor, int origin)
    {
        if (Free(grid, origins, line, start, length, origin)) return start;

        // Right-anchored text belongs against the right margin, so it retreats leftwards;
        // everything else shifts the way it reads.
        var step = anchor == CniAnchor.Right ? -1 : 1;

        for (var shift = 1; shift < Columns; shift++)
        {
            var candidate = start + shift * step;
            if (candidate < 0 || candidate + length > Columns) break;
            if (Free(grid, origins, line, candidate, length, origin)) return candidate;
        }

        return start;
    }

    /// <summary>
    /// Free for this element: either empty, or already holding text drawn from the same anchor.
    /// The second case is a field the sim deliberately draws over another — COMM TUNE INDEX
    /// puts the highlighted "GUARD" on top of the identifier it replaces — and moving it aside
    /// would show both, side by side, where the cockpit shows one.
    /// </summary>
    private static bool Free(char[,] grid, int[,] origins, int line, int start, int length, int origin)
    {
        for (var i = 0; i < length; i++)
        {
            var col = start + i;
            if (col < 0 || col >= Columns) continue;
            if (grid[line, col] == ' ') continue;
            if (origins[line, col] != origin) return false;
        }
        return true;
    }

    private static List<CniRun> ToRuns(char[,] grid, bool[,] small, bool[,] invert)
    {
        var runs = new List<CniRun>();

        for (var line = 0; line < Lines; line++)
        {
            var col = 0;
            while (col < Columns)
            {
                if (!Occupied(grid, invert, line, col))
                {
                    col++;
                    continue;
                }

                var start = col;
                var isSmall = small[line, col];
                var isInvert = invert[line, col];
                var text = new StringBuilder();

                while (col < Columns && Occupied(grid, invert, line, col)
                       && small[line, col] == isSmall && invert[line, col] == isInvert)
                {
                    text.Append(grid[line, col]);
                    col++;
                }

                runs.Add(new CniRun(line, start, text.ToString(), isSmall, isInvert));
            }
        }

        return runs;
    }

    /// <summary>
    /// Whether a cell belongs to a run. A blank normally ends one, but a blank inside inverted
    /// text does not: the inverted material covers the whole string the module draws it behind,
    /// spaces included, and treating them as gaps would break the bar into pieces.
    /// </summary>
    private static bool Occupied(char[,] grid, bool[,] invert, int line, int col) =>
        grid[line, col] != ' ' || invert[line, col];

    /// <summary>
    /// Translates the two repurposed characters. Anything else passes through: the CNI sends
    /// printable ASCII only, so unlike the CDNU there is nothing to blank out.
    /// </summary>
    private static string MapGlyphs(string? raw)
    {
        if (string.IsNullOrEmpty(raw)) return string.Empty;

        return string.Create(raw.Length, raw, static (dst, src) =>
        {
            for (var i = 0; i < src.Length; i++)
                dst[i] = Glyphs.TryGetValue(src[i], out var glyph) ? glyph : src[i];
        });
    }
}
