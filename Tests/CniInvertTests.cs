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
