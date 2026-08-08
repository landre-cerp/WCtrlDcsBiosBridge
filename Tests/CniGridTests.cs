using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class CniGridTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    private static string[] Render(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);
        return CniGrid.ToLines(CniGrid.Render(data, page));
    }

    /// <summary>
    /// Several elements can share one anchor and space themselves apart with their own leading
    /// blanks. TOLD INIT's RCR/RSC line is three of them issued at column 0 — "23", "  /" and
    /// "    0" — and the blanks of the later ones were landing on the characters of the earlier,
    /// rubbing the slash off a line the cockpit reads as 23/ 0.
    ///
    /// Ink over ink is still allowed: that is the deliberate overdraw COMM TUNE INDEX does when
    /// it puts a highlighted GUARD on top of the identifier it replaces.
    /// </summary>
    [Fact]
    public void SpacesDoNotRubOutWhatIsAlreadyOnTheLine()
    {
        Assert.StartsWith("23/ 0 ", Render("told-init-1of2")[10]);
    }

    /// <summary>
    /// A leg the route is broken at draws two things and no more: the banner, alone on its line,
    /// and the leg's own name on the line below. Everything the FMS cannot compute for it — the
    /// bearing, the distance, the time, the speed, the altitude — is simply absent.
    ///
    /// Read as an ordinary leg, the dashes standing in for that name went to the distance beside
    /// the banner, which reads the same and comes first, and the whole route below moved up a
    /// line: the aircraft's approach speeds and altitudes against the wrong waypoints.
    /// </summary>
    [Fact]
    public void ABrokenLegDrawsItsBannerAndItsNameAndNothingElse()
    {
        var lines = Render("legs-1");

        Assert.Equal(">> DISCONTINUITY <<", lines[1].Trim());
        Assert.Equal("-----", lines[2].Trim());

        // And the route below it starts where it should, down to the last altitude — the one
        // the alignment used to throw overboard to keep a blank.
        Assert.Equal("LAMU3B ---°       ----:--".Replace("\u00b0", "°"), lines[3].TrimEnd());
        Assert.Equal("RW31           210/ 01574", lines[8].TrimEnd());
    }

    [Theory]
    [MemberData(nameof(FixtureNames))]
    public void GridIsAlwaysFourteenByTwentyFive(string fixture)
    {
        var lines = Render(fixture);

        Assert.Equal(CniGrid.Lines, lines.Length);
        Assert.All(lines, l => Assert.Equal(CniGrid.Columns, l.Length));
    }

    /// <summary>
    /// Nothing may run off either edge. This is the failure the column mapping got wrong at
    /// first: placing text relative to the centre of a grid narrower than the page quietly ate
    /// a character on each side, turning "&lt;DATA XFR" into "DATA XFR". Values overwriting one
    /// another is a separate matter and legitimate — the sim draws in the same order — so this
    /// checks placement rather than the final screen.
    /// </summary>
    [Theory]
    [MemberData(nameof(FixtureNames))]
    public void NothingIsClippedByTheGridEdges(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);

        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var matched = CniBlockMatcher.Align(blocks, page!.Slots);

        var clipped = new List<string>();

        for (var i = 0; i < blocks.Count; i++)
        {
            var slot = matched[i];
            var text = blocks[i].V;
            if (slot is null || !slot.IsPlaceable || string.IsNullOrEmpty(text)) continue;

            // A rule of dashes is drawn wider than the display on purpose; trimming it is
            // the one loss that costs nothing.
            if (text.Length > CniGrid.Columns) continue;

            var start = CniGrid.StartColumn(slot.Anchor, slot.Col!.Value, text.Length);
            if (start < 0 || start + text.Length > CniGrid.Columns)
                clipped.Add($"'{text}' anchor={slot.Anchor} col={slot.Col} -> {start}..{start + text.Length}");
        }

        Assert.True(clipped.Count == 0,
            $"{page.Name}: {string.Join(" | ", clipped)}");
    }

    /// <summary>
    /// A whole page, locked in. Fine placement cannot be proved without holding it against
    /// the cockpit, so this does not claim the layout is right — it claims it has not moved.
    /// </summary>
    [Fact]
    public void IndexPageRendersAsRecorded()
    {
        // Recorded at 25 columns. The space in "PERF WGT 1" is the compression going away:
        // squeezed onto 24 columns the label and its number landed on the same cell, and the
        // reflow closed the gap to keep them apart.
        var expected = new[]
        {
            "1          INDEX      1/2",
            "                         ",
            "<POWER UP        ROUTE 1>",
            "                         ",
            "<PERF WGT 1      ROUTE 2>",
            "                         ",
            "<PERF CLB 1      RT LIST>",
            "                         ",
            "<PERF CRZ 1    DEP/ARR 1>",
            "                         ",
            "<PERF DES 1     DATA XFR>",
            "                         ",
            "<ZEROIZE        PROGRESS>",
            // The scratchpad, which add_scratch() puts at the left of the bottom line.
            "243.000                  ",
        };

        Assert.Equal(expected, Render("232316-058"));
    }

    /// <summary>
    /// Roughly centred, not exactly: the module centres a title on column 11 while the page
    /// runs to 25, so it sits about a column and a half to the left of the middle. The margin
    /// here is wide enough to catch a title landing on the wrong side of the display and no
    /// tighter, because the offset is the module's and not something to correct.
    /// </summary>
    [Fact]
    public void TitleIsCentredOnTheTopLine()
    {
        var lines = Render("232316-006");   // COMM TUNE U1

        Assert.Contains("COMM TUNE U1", lines[0]);

        var start = lines[0].IndexOf("COMM TUNE U1", StringComparison.Ordinal);
        var end = CniGrid.Columns - (start + "COMM TUNE U1".Length);
        Assert.True(Math.Abs(start - end) <= 3, $"titre decentre: {start} a gauche, {end} a droite");
    }

    [Fact]
    public void LeftAnchoredLabelStartsAtTheLeftEdge()
    {
        var lines = Render("232316-006");
        var row = lines.First(l => l.Contains("IDENT", StringComparison.Ordinal));

        Assert.StartsWith("IDENT", row.TrimEnd());
    }

    [Fact]
    public void RightAnchoredLabelEndsAtTheRightEdge()
    {
        var lines = Render("232316-058");   // INDEX, line-select labels down the right side
        var row = lines.First(l => l.Contains("ROUTE 1>", StringComparison.Ordinal));

        Assert.EndsWith("ROUTE 1>", row);
    }

    /// <summary>
    /// The CNI font puts a degree sign on '^'. Left alone, coordinates read "N42^14.56".
    /// </summary>
    [Fact]
    public void DegreeGlyphIsTranslated()
    {
        // INAV1 CTRL SENSORS shows the present position, "N42^14.56 E042^02.12".
        var lines = Render("232316-027");
        var screen = string.Join("\n", lines);

        Assert.DoesNotContain("^", screen, StringComparison.Ordinal);
        Assert.Contains("°", screen, StringComparison.Ordinal);
    }

    [Fact]
    public void NoDataRendersBlank()
    {
        var lines = CniGrid.ToLines(CniGrid.Render(null, null));

        Assert.All(lines, l => Assert.Equal(new string(' ', CniGrid.Columns), l));
    }

    /// <summary>
    /// An unrecognised page must not be drawn against another page's layout: every field
    /// would land somewhere plausible and wrong.
    /// </summary>
    [Fact]
    public void UnresolvedPageRendersBlank()
    {
        var lines = CniGrid.ToLines(CniGrid.Render(CniFixtures.Load("dump-001"), null));

        Assert.All(lines, l => Assert.Equal(new string(' ', CniGrid.Columns), l));
    }

    public static TheoryData<string> FixtureNames()
    {
        var data = new TheoryData<string>();
        foreach (var name in CniFixtures.Names) data.Add(name);
        return data;
    }
}
