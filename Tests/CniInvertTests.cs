using WCtrlDcsBiosBridge.Services;
using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// The highlight is the value on a rotary field, so putting it in the wrong place says
/// something false rather than merely looking untidy. These pin down when it is drawn.
/// </summary>
public class CniInvertTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    private static IReadOnlyList<CniRun> Render(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);
        return CniGrid.Render(data, page);
    }

    private static CniPage Page(string name) =>
        CniFixtures.Schema.Pages.Single(p => p.Name == name);

    [Fact]
    public void PairedVariantsAreTheTwoFormsOfOneElement()
    {
        foreach (var page in CniFixtures.Schema.Pages)
        {
            foreach (var slot in page.Slots.Where(s => s.Counterpart is not null))
            {
                var other = slot.Counterpart!;

                Assert.Same(slot, other.Counterpart);

                // A pair is either the two states of a field or, where the module named no
                // state, the highlighted and plain forms of one element.
                Assert.True(slot.IsSelected != other.IsSelected || slot.IsInvert != other.IsInvert);
            }
        }
    }

    /// <summary>
    /// An element alone at its spot is taken as proof of its field's state, so an element left
    /// alone by a pairing that should have caught it becomes a highlight that is simply made up
    /// — MODE S sat on ON for a whole flight that way, because iff_mode_s_out ends in neither
    /// _on nor _off and its branch went unlabelled.
    ///
    /// Nine slots are genuinely alone: the GUARD word in the IDENT column of UHF1 and UHF2, and
    /// seven labels on the drop-zone pages. A module update that renames a controller shows up
    /// here rather than on the device.
    /// </summary>
    [Fact]
    public void OnlyTheKnownSingletonsStandAloneInATwoStateField()
    {
        var alone = new List<string>();

        foreach (var page in CniFixtures.Schema.Pages)
        {
            foreach (var slot in page.Slots.Where(s => s is { FieldHasTwoStates: true }
                                                       && s.Counterpart is null
                                                       && s.Line is not null))
            {
                alone.Add($"{page.Name}/{slot.Controller}/{slot.Value ?? "<dyn>"}");
            }
        }

        Assert.Equal(new[]
        {
            "CARP_INIT_4/carp_stg_2/ MIN DROP HT",
            "CARP_INIT_4/carp_stg_2/ RQD CLNC HT",
            "CARP_INIT_4/carp_stg_2/ACTUATION ALTITUDE",
            "CARP_INIT_4/carp_stg_2/DZ ELEV ",
            "CARP_INIT_4/carp_stg_2/OBSTR ELEV ",
            "CARP_INIT_6/carp_drive_yes/ DRIVE DIST",
            "CARP_INIT_6/carp_drive_yes/ HARP",
            "UHF1/uhf1_guard/GUARD",
            "UHF2/uhf2_guard/GUARD",
        }, alone.OrderBy(x => x, StringComparer.Ordinal).ToArray());
    }

    /// <summary>
    /// A slot the pairing did not reach keeps its highlight unconditionally, so it had better be
    /// an element the module really does build in one form only. INAV CTRL fell through for a
    /// whole release because it names its branches nav_ctrl_1_gps1_sol_on_p — state in the
    /// middle, seat at the end — and thirteen elements on that page lit up regardless of what
    /// the crew had selected.
    ///
    /// These twelve are the genuine single-form families: the scratchpad message, the page and
    /// programme counters, two status readouts, and the GUARD word COMM TUNE INDEX draws through
    /// a controller of its own. Anything new appearing here is a naming convention the schema
    /// does not understand, and it will be showing a highlight that means nothing.
    /// </summary>
    [Fact]
    public void OnlyKnownSingleFormFamiliesAreHighlightedUnconditionally()
    {
        var families = new SortedSet<string>(StringComparer.Ordinal);

        foreach (var page in CniFixtures.Schema.Pages)
        {
            foreach (var slot in page.Slots)
            {
                if (!slot.IsInvert || !slot.IsPlaceable) continue;
                if (slot.Counterpart is not null || slot.FieldHasTwoStates) continue;

                families.Add(slot.Controller ?? slot.Source ?? "<none>");
            }
        }

        Assert.Equal(new[]
        {
            "active_rt_num", "carp_num", "def_sys_rwr_ofp_status", "def_sys_wow_ord_status",
            "lz_num", "rndz_num", "sar_prog_num", "scratch_error",
            "uhf1_guard_on2", "uhf2_guard_on2", "wt_bal_delete_verify", "wt_bal_op_ldg",
        }, families.ToArray());
    }

    /// <summary>
    /// The failure this whole pass exists to prevent. Before the fields were paired, PWR came
    /// out with OFF and ON both highlighted and ADF/MN/BTH with two of three — displays the
    /// aircraft cannot produce.
    /// </summary>
    [Theory]
    [MemberData(nameof(FixtureNames))]
    public void NoFieldEverShowsTwoMembersHighlighted(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        if (page is null) return;

        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var matched = CniBlockMatcher.Align(blocks, page.Slots);
        CniVariants.Apply(matched, page);

        // Two-state fields only: a single-state one draws the same highlighted value wherever
        // it likes, and INDEX prints active_rt_num five times down the page. Per line, because
        // one field legitimately lights more than one place: the GUARD word in the IDENT column
        // and the GUARD toggle share uhf1_guard and are both drawn when it is on. What cannot
        // happen is two of a field's words lit side by side.
        var highlighted = matched
            .Where(s => s is { Controller: not null, FieldHasTwoStates: true } && s.IsInvert)
            .GroupBy(s => (s!.Controller, s.Line));

        Assert.All(highlighted, g => Assert.Single(g));
    }

    /// <summary>
    /// COMM TUNE U1 draws the word GUARD in the IDENT column only while the GUARD toggle is on,
    /// and both run off uhf1_guard. The capture has that word, so the toggle three lines below
    /// must come out on ON — which is what the cockpit shows.
    /// </summary>
    [Fact]
    public void GuardWordInTheIdentColumnSettlesTheToggle()
    {
        var runs = Render("232316-006");

        Assert.Contains(runs, r => r.Invert && r.Line == 2 && r.Text.Trim() == "GUARD");

        var toggle = runs.Single(r => r.Invert && r.Line == 4);
        Assert.Equal("ON", toggle.Text.Trim());
    }

    /// <summary>
    /// ADF spells itself differently in its two states — "ADF" selected against "ADF/ " plain —
    /// so the matcher has already had to choose on the text and the choice can be trusted.
    /// </summary>
    [Theory]
    [InlineData("232316-006", true)]
    [InlineData("232316-015", false)]
    [InlineData("232316-016", false)]
    public void RadioModeFollowsTheTextTheSimSent(string fixture, bool adfSelected)
    {
        var runs = Render(fixture);
        var highlighted = runs.Any(r => r.Invert && r.Line == 5 && r.Text.Trim().StartsWith("ADF"));

        Assert.Equal(adfSelected, highlighted);
    }

    /// <summary>
    /// PWR builds both its words alike, so nothing on the wire says which is lit. Neither is.
    /// </summary>
    [Fact]
    public void FieldWithNoEvidenceIsDrawnUnselected()
    {
        var runs = Render("232316-006");

        Assert.DoesNotContain(runs, r => r.Invert && r.Line == 2 && r.Column > 16);
    }

    /// <summary>
    /// Both halves of a paired field are then drawn at the same size, since the size is the
    /// other half of the same claim.
    /// </summary>
    [Fact]
    public void UnselectedFieldDoesNotClaimAStateThroughItsSize()
    {
        var words = Render("232316-006")
            .Where(r => r.Line == 2 && r.Column > 16)
            .Where(r => r.Text.Trim() is "OFF" or "ON")
            .ToList();

        Assert.Equal(2, words.Count);
        Assert.All(words, r => Assert.True(r.Small));
    }

    /// <summary>
    /// COMM TUNE U1 prints the frequency it is tuned to, and one radio device is transmitting
    /// there. That device answers is_on(), which is the state the page cannot express — so the
    /// PWR toggle can be lit without either end naming a device number.
    /// </summary>
    [Theory]
    [InlineData(true,  "ON")]
    [InlineData(false, "OFF")]
    public void RadioPowerLightsTheWordItBelongsTo(bool powered, string expected)
    {
        var data = CniFixtures.Load("232316-006") with
        {
            Radios = new List<RadioState> { new(7, 243_000, powered) },
        };

        var runs = CniGrid.Render(data, Resolver.Resolve(data));
        var pwr = runs.Single(r => r.Invert && r.Line == 2 && r.Column > 16);

        Assert.Equal(expected, pwr.Text.Trim());
        Assert.False(pwr.Small);
    }

    /// <summary>
    /// A radio the page is not tuned to says nothing about it. 264.000 is UHF2, three lines down
    /// the preset list on this page and not what its PWR toggle is about.
    /// </summary>
    [Fact]
    public void RadioOnAnotherFrequencyDoesNotLightThePage()
    {
        var data = CniFixtures.Load("232316-016") with
        {
            Radios = new List<RadioState> { new(9, 264_000, true) },
        };

        var runs = CniGrid.Render(data, Resolver.Resolve(data));

        Assert.DoesNotContain(runs, r => r.Invert && r.Line == 2 && r.Column > 16);
    }

    /// <summary>
    /// The corner digit is the page's own INAV number, boxed when the aircraft has settled on
    /// that INAV. The pilot's CNI prints 1, so it lights on solution 1 and goes plain on 2 —
    /// which is what the copilot's 2 does in mirror.
    /// </summary>
    [Theory]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(null, false)]
    public void CornerDigitFollowsTheShipSolution(int? solution, bool boxed)
    {
        var data = CniFixtures.Load("dump-001") with { ShipSolution = solution };

        var runs = CniGrid.Render(data, Resolver.Resolve(data));
        var corner = runs.Single(r => r.Line == 0 && r.Column == 0);

        Assert.Equal("1", corner.Text);
        Assert.Equal(boxed, corner.Invert);
    }

    /// <summary>
    /// A pair the module marks by size alone has no highlight to drop, so the smaller of its two
    /// forms is not the quieter one — it is simply the other state. POWER UP's MSTR AV ON and
    /// AUTONAV are drawn full size and redrawn small once set, and taking the small form for the
    /// neutral one had the device saying the crew had already set them.
    /// </summary>
    [Fact]
    public void PairMarkedBySizeAloneDrawsTheUnselectedForm()
    {
        var runs = Render("dump-001");

        var master = runs.Where(r => r.Line == 8).ToList();
        Assert.NotEmpty(master);
        Assert.All(master, r => Assert.False(r.Small));

        var autonav = runs.Single(r => r.Line == 10 && r.Text.Trim() == "AUTONAV");
        Assert.False(autonav.Small);
    }

    /// <summary>
    /// The NAV DB lines are marked by size too, the other way round — the loaded database is the
    /// large one — so the unselected form is the small one on both, and neither claims to be
    /// loaded. This is the reading that had 09SEP09OCT24 on the device while the cockpit had
    /// 12AUG10SEP25.
    /// </summary>
    [Fact]
    public void NeitherNavDatabaseClaimsToBeTheLoadedOne()
    {
        var runs = Render("dump-001");

        Assert.True(runs.Single(r => r.Text.Trim() == "12AUG10SEP25").Small);
        Assert.True(runs.Single(r => r.Text.Trim() == "09SEP09OCT24").Small);
    }

    private static string Row(IReadOnlyList<CniRun> runs, int line) =>
        CniGrid.ToLines(runs)[line].Trim();

    /// <summary>
    /// A table row is built as every highlighted cell followed by every plain one, so with a
    /// column selected the sim sends that column's value first and the rest in column order:
    /// LANDING DATA's approach speeds arrive "126 153 135" for a row that reads 153 135 126.
    ///
    /// Read as though the wire were column order, the row came out 126 153 135 — the aircraft's
    /// landing speeds against the wrong flap settings. Worse than a missing highlight, and the
    /// reason this page was looked at at all.
    /// </summary>
    [Theory]
    [InlineData("landing-1of2-flaps100")]
    [InlineData("landing-1of2-flaps50")]
    [InlineData("landing-1of2-plain")]
    public void TableRowsKeepTheirColumnsWhicheverIsSelected(string fixture)
    {
        var runs = Render(fixture);

        Assert.Equal("FLAPS     0  50  100 MAX", Row(runs, 2));
        Assert.Equal("APP    153 135 126 ---", Row(runs, 4));
        Assert.Equal("THR    143 125 116  99", Row(runs, 6));
        Assert.Equal("TD     137 119 110  93", Row(runs, 8));
    }

    /// <summary>
    /// And the column the header points at is the one the rows beneath it light.
    ///
    /// The header is the only row of the four made of literals, which is what makes it readable:
    /// "0", "50" and "100" are spelled the same in both halves of their pair, but no plain
    /// reading can put "100" ahead of the two that follow it, so the matcher has to seat it on
    /// the highlighted half. The three rows of figures have nothing of their own to go on and
    /// take the answer from the column.
    /// </summary>
    [Fact]
    public void SelectedColumnIsReadOffTheHeaderAndCarriesDownTheTable()
    {
        var lit = Render("landing-1of2-flaps100")
            .Where(r => r.Invert)
            .Select(r => $"L{r.Line}:{r.Text.Trim()}")
            .ToArray();

        Assert.Equal(new[] { "L2:100", "L4:126", "L6:116", "L8:110" }, lit);

        // The middle column is the one that caught the last of it. Its cell is drawn highlighted,
        // so the plain half of that same cell is not drawn at all — and until that was said
        // outright, the value of the column beyond it took the plain half instead, two figures
        // in one cell and the last column blank.
        var middle = Render("landing-1of2-flaps50")
            .Where(r => r.Invert)
            .Select(r => $"L{r.Line}:{r.Text.Trim()}")
            .ToArray();

        Assert.Equal(new[] { "L2:50", "L4:135", "L6:125", "L8:119" }, middle);

        // The same page with no column selected sends its values in column order and claims
        // nothing, which is the reading that must not acquire a highlight of its own.
        Assert.DoesNotContain(Render("landing-1of2-plain"), r => r.Invert);
    }

    /// <summary>
    /// The other half of the same fault. A pair's two halves sit in one cell, and filling both
    /// drew the plain one over the highlighted one and lost the value underneath: LANDING DATA
    /// 2/2 showed the ground roll figures on the 50 FT LDG line, and TOLD INDEX three of its six
    /// lines with the fourth written over the first.
    /// </summary>
    [Fact]
    public void BothHalvesOfACellAreNeverFilledAtOnce()
    {
        var landing = Render("landing-2of2-flaps0");

        Assert.Equal("5546  4440  3867  -----", Row(landing, 4));
        Assert.Equal("2645  1922  1648   1241", Row(landing, 6));

        var told = Render("told-index");

        Assert.Equal("<TOLD INIT       UGKS/09", Row(told, 2));
        foreach (var line in new[] { 4, 6, 8, 10, 12 })
            Assert.EndsWith("-----/---", Row(told, line));
    }

    /// <summary>
    /// The bottom-line message is the highlight no variant competes with.
    /// </summary>
    [Fact]
    public void BottomLineMessageIsHighlighted()
    {
        var message = Assert.Single(Render("dump-013").Where(r => r.Invert));

        Assert.Equal(13, message.Line);
        Assert.Equal("NO ACTIVE ROUTE", message.Text.Trim());
    }

    /// <summary>
    /// A highlight runs behind the whole string. Splitting it on its own spaces would draw the
    /// message as three blocks with gaps between the words.
    /// </summary>
    [Fact]
    public void HighlightIsNotBrokenByTheSpacesInsideIt()
    {
        Assert.Single(Render("dump-013").Where(r => r.Invert && r.Line == 13));
    }

    [Theory]
    [InlineData("232316-027")]
    [InlineData("232316-057")]
    [InlineData("dump-001")]
    public void PagesWithNothingToHighlightRenderPlain(string fixture)
    {
        Assert.DoesNotContain(Render(fixture), r => r.Invert);
    }

    public static TheoryData<string> FixtureNames()
    {
        var data = new TheoryData<string>();
        foreach (var name in CniFixtures.Names) data.Add(name);
        return data;
    }
}
