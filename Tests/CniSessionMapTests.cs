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

        // ADF spells itself differently in its two forms, so its value moves with its element
        // just as the sim does it — that is what lets the matcher settle ADF on text alone.
        static CniData Uhf1(bool adfLit, string bth, string mn, string sep)
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

        var onAdf = Uhf1(true,  "{BTH-OFF}", "{MN-OFF}", "{SEP-A}");
        var onBth = Uhf1(false, "{BTH-LIT}", "{MN-OFF}", "{SEP-B}");
        var onMn  = Uhf1(false, "{BTH-OFF}", "{MN-LIT}", "{SEP-A}");

        string? Lit(CniData d)
        {
            var runs = CniGrid.Render(d, Resolver.Resolve(d), map);
            return runs.Where(r => r.Invert && r.Text.Trim() is "ADF" or "ADF/" or "BTH" or "MN")
                       .Select(r => r.Text.Trim())
                       .FirstOrDefault();
        }

        Lit(onAdf); Lit(onBth); Lit(onMn);

        Assert.Equal("BTH", Lit(onBth));
        Assert.Equal("MN",  Lit(onMn));
    }
}
