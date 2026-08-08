using WCtrlDcsBiosBridge.Aircrafts.C130J;
using WCtrlDcsBiosBridge.Services;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Replays the three states a probe session caught while the crew turned POWER UP's alignment
/// rotary through LAST, REF and GPS, with the element names exactly as the sim issued them.
///
/// Nothing outside the CNI moved with that rotary — every indicator was checked — so if the map
/// cannot read it off the turns, the field cannot be read at all.
/// </summary>
public class CniSessionMapTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    private const string GpsPlain = "{19753290-0000-0000-0000-000000000001}";
    private const string GpsLit   = "{757F2030-0000-0000-0000-000000000002}";
    private const string LastLit  = "{C4C1EA9A-0000-0000-0000-000000000003}";
    private const string LastPlain = "{56E6A019-0000-0000-0000-000000000004}";
    private const string RefPlain = "{EDFA916F-0000-0000-0000-000000000005}";
    private const string RefLit   = "{D00E7CF6-0000-0000-0000-000000000006}";

    /// <summary>
    /// Rewrites the capture's own element names for the three alignment words, so a state can be
    /// posed by saying which element drew each of them.
    /// </summary>
    private static CniData PowerUpWith(string gps, string last, string reference)
    {
        var data = CniFixtures.Load("dump-001");

        var blocks = data.Blocks!.Select(b => b.V switch
        {
            "GPS"  => b with { K = gps },
            "LAST" => b with { K = last },
            "REF"  => b with { K = reference },
            _      => b,
        }).ToList();

        return data with { Blocks = blocks };
    }

    private static bool AlignmentLit(CniSessionMap map, CniData data, string word)
    {
        var page = Resolver.Resolve(data)!;
        var runs = CniGrid.Render(data, page, map);

        return runs.Any(r => r.Invert && r.Text.Trim() == word);
    }

    /// <summary>
    /// The rotary alone settles itself, in two turns and without help from anywhere else.
    /// </summary>
    [Fact]
    public void TwoTurnsSettleTheWholeRotary()
    {
        var map = new CniSessionMap();

        var onLast = PowerUpWith(GpsPlain, LastLit, RefPlain);
        var onRef  = PowerUpWith(GpsPlain, LastPlain, RefLit);
        var onGps  = PowerUpWith(GpsLit,   LastPlain, RefPlain);

        // Nothing is known of a rotary that has not moved.
        Assert.False(AlignmentLit(map, onLast, "LAST"));

        // First turn: GPS held, so GPS is not the lit one either side of it.
        Assert.False(AlignmentLit(map, onRef, "REF"));

        // Second turn: LAST held. It was LAST or REF that was lit before, so it was REF — and
        // by elimination GPS now.
        Assert.True(AlignmentLit(map, onGps, "GPS"));

        // And what was learned holds when the crew turns back.
        Assert.True(AlignmentLit(map, onRef, "REF"));
        Assert.True(AlignmentLit(map, onLast, "LAST"));
    }

    /// <summary>
    /// One member lit at a time, however many turns have gone by. Two would be a display the
    /// aircraft cannot produce.
    /// </summary>
    [Fact]
    public void NeverLightsTwoPositionsAtOnce()
    {
        var map = new CniSessionMap();
        var states = new[]
        {
            PowerUpWith(GpsPlain, LastLit, RefPlain),
            PowerUpWith(GpsPlain, LastPlain, RefLit),
            PowerUpWith(GpsLit,   LastPlain, RefPlain),
            PowerUpWith(GpsPlain, LastLit, RefPlain),
        };

        foreach (var state in states)
        {
            var runs = CniGrid.Render(state, Resolver.Resolve(state), map);
            var words = runs.Count(r => r.Invert && r.Text.Trim() is "GPS" or "LAST" or "REF");

            Assert.InRange(words, 0, 1);
        }
    }

    /// <summary>
    /// Fresh elements mean a rebuilt page, and everything learned about the old ones goes with
    /// them — a GUID is only good for the session that issued it.
    /// </summary>
    [Fact]
    public void ForgetsWhenEveryElementIsReplaced()
    {
        var map = new CniSessionMap();

        AlignmentLit(map, PowerUpWith(GpsPlain, LastLit, RefPlain), "LAST");
        AlignmentLit(map, PowerUpWith(GpsPlain, LastPlain, RefLit), "REF");
        AlignmentLit(map, PowerUpWith(GpsLit, LastPlain, RefPlain), "GPS");

        var rebuilt = PowerUpWith("{AAAA0001-0000-0000-0000-00000000000A}",
                                  "{AAAA0002-0000-0000-0000-00000000000B}",
                                  "{AAAA0003-0000-0000-0000-00000000000C}");

        var runs = CniGrid.Render(rebuilt, Resolver.Resolve(rebuilt), map);

        Assert.DoesNotContain(runs, r => r.Invert && r.Text.Trim() is "GPS" or "LAST" or "REF");
    }

    /// <summary>
    /// POWER UP's alignment rotary is one of the ones the schema recognises, and it has the
    /// three members the reasoning needs.
    /// </summary>
    [Fact]
    public void AlignmentRotaryIsRecognisedAsASelector()
    {
        var page = CniFixtures.Schema.Pages.Single(p => p.Name == "POWER_UP");
        var rotary = page.Selectors.Single(s => s.Key == "power_up_align");

        Assert.Equal(new[] { "power_up_align_gps", "power_up_align_last", "power_up_align_ref" },
                     rotary.Members);
    }

    /// <summary>
    /// COMM TUNE U1's ADF/MN/BTH, cycled the way the crew does. Every position must light in
    /// its turn once the rotary has been round twice.
    /// </summary>
    [Fact]
    public void EveryPositionOfTheRadioModeRotaryLightsInTurn()
    {
        var map = new CniSessionMap();

        var onAdf = Uhf1RadioMode(true,  "{BTH-OFF}", "{MN-OFF}", "{SEP-A}");
        var onBth = Uhf1RadioMode(false, "{BTH-LIT}", "{MN-OFF}", "{SEP-B}");
        var onMn  = Uhf1RadioMode(false, "{BTH-OFF}", "{MN-LIT}", "{SEP-A}");

        RadioMode(map, onAdf); RadioMode(map, onBth); RadioMode(map, onMn);

        Assert.Equal("BTH", RadioMode(map, onBth));
        Assert.Equal("MN",  RadioMode(map, onMn));
    }

    /// <summary>
    /// ADF is the one member of that rotary the page settles by itself, spelling itself "ADF/ "
    /// when it is not the selected one. Knowing which element draws the selected ADF is knowing
    /// that the elements drawing MN and BTH beside it are not selected — so the very first turn
    /// away from ADF says which of the two it went to, where turn-watching alone needed a second.
    /// </summary>
    [Fact]
    public void TextEvidenceOnOneMemberSettlesTheOnesBesideIt()
    {
        var map = new CniSessionMap();

        var onAdf = Uhf1RadioMode(adfLit: true,  bth: "{BTH-A}", mn: "{MN-A}", sep: "{SEP-A}");
        var onBth = Uhf1RadioMode(adfLit: false, bth: "{BTH-B}", mn: "{MN-A}", sep: "{SEP-B}");

        Assert.Equal("ADF", RadioMode(map, onAdf));
        Assert.Equal("BTH", RadioMode(map, onBth));
    }

    /// <summary>
    /// The PWR toggle is settled by the radio device, which the page can only be paired with
    /// while it is printing the frequency that device is on. Kept against the element rather than
    /// the frame, the reading survives the page that gives no frequency to pair on — and the
    /// element the sim swaps to when the crew cuts the power is, by the same token, the other one.
    /// </summary>
    [Fact]
    public void RadioPowerOutlivesTheFrameThatProvedIt()
    {
        var map = new CniSessionMap();

        var tuned = CniFixtures.Load("232316-006") with
        {
            Radios = new List<RadioState> { new(7, 243_000, true) },
        };

        Assert.Equal("ON", Power(map, tuned));

        // Same session, same elements behind PWR, and nothing to pair a radio with.
        Assert.Equal("ON", Power(map, CniFixtures.Load("232316-016")));

        // Same session, other elements behind PWR. A field draws one of two forms and no more.
        Assert.Equal("OFF", Power(map, CniFixtures.Load("232316-015")));
    }

    /// <summary>
    /// The reading is about elements, so it has to end when they do. A rebuilt page issues fresh
    /// GUIDs for everything, and an identity that is unknown because it is new would otherwise
    /// read — through the very rule that makes the map useful — as the opposite of the one on
    /// file, lighting the wrong word with full confidence.
    /// </summary>
    [Fact]
    public void RebuiltPageIsNotReadThroughTheElementsItReplaced()
    {
        var map = new CniSessionMap();

        var tuned = CniFixtures.Load("232316-006") with
        {
            Radios = new List<RadioState> { new(7, 243_000, true) },
        };

        Assert.Equal("ON", Power(map, tuned));
        Assert.Null(Power(map, Rebuilt(CniFixtures.Load("232316-006"))));
    }

    /// <summary>COMM TUNE U1's PWR word the device is showing lit, or null for neither.</summary>
    private static string? Power(CniSessionMap map, CniData data) =>
        CniGrid.Render(data, Resolver.Resolve(data), map)
               .Where(r => r.Invert && r.Line == 2 && r.Column > 16)
               .Select(r => r.Text.Trim())
               .FirstOrDefault();

    /// <summary>COMM TUNE U1's lit ADF/MN/BTH position, or null for none.</summary>
    private static string? RadioMode(CniSessionMap map, CniData data) =>
        CniGrid.Render(data, Resolver.Resolve(data), map)
               .Where(r => r.Invert && r.Text.Trim() is "ADF" or "ADF/" or "BTH" or "MN")
               .Select(r => r.Text.Trim())
               .FirstOrDefault();

    /// <summary>
    /// COMM TUNE U1 with its ADF/MN/BTH rotary posed by naming the element drawing each word.
    /// ADF's value moves with its element the way the sim does it, which is what lets the matcher
    /// settle that member on text alone.
    /// </summary>
    private static CniData Uhf1RadioMode(bool adfLit, string bth, string mn, string sep)
    {
        var data = CniFixtures.Load("232316-006");

        var blocks = data.Blocks!.Select(b => b.V switch
        {
            "ADF" => b with { K = adfLit ? "{ADF-LIT}" : "{ADF-PLAIN}",
                              V = adfLit ? "ADF" : "ADF/ " },
            "BTH" => b with { K = bth },
            "MN"  => b with { K = mn },
            "/"   => b with { K = sep },
            _     => b,
        }).ToList();

        return data with { Blocks = blocks };
    }

    /// <summary>
    /// The same page as the sim builds it a second time: every value where it was, every element
    /// name new. Only the two the sim names itself keep theirs, as they do in a real rebuild.
    /// </summary>
    private static CniData Rebuilt(CniData data)
    {
        var issued = 0;

        List<CniBlock> Reissue(List<CniBlock> blocks) => blocks.Select(b => b with
        {
            K = b.K is "cni_title" or "cni_scratchpad"
                ? b.K
                : $"{{REBUILT-{++issued:0000}}}",
            C = b.C is null ? null : Reissue(b.C),
        }).ToList();

        return data with { Blocks = Reissue(data.Blocks!) };
    }
}
