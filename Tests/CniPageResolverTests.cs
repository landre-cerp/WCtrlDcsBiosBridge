using WCtrlDcsBiosBridge.Services;
using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class CniPageResolverTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    [Theory]
    [InlineData("232316-006", "UHF1")]              // COMM TUNE U1
    [InlineData("232316-003", "COMM_TUNE_1")]       // COMM TUNE INDEX
    [InlineData("232316-027", "NAV_CTRL_2")]        // title built from "INAV%d CTRL SENSORS"
    [InlineData("232316-077", "LZ_INIT_1")]         // title built from "%sLZ %2d INIT"
    [InlineData("232316-056", "MSN_IDX")]
    [InlineData("232316-057", "TOLD_IDX")]
    [InlineData("dump-001", "POWER_UP")]
    [InlineData("dump-005", "IFF_1")]               // three pages share the title "IFF"
    [InlineData("carp-init-1of5", "CARP_INIT_1")]   // six pages share "CARP %d INIT"
    [InlineData("carp-init-2of5", "CARP_INIT_2")]
    [InlineData("carp-init-5of5", "CARP_INIT_5")]
    [InlineData("carp-prog-1of2", "CARP_PROG_1")]
    [InlineData("carp-prog-2of2", "CARP_PROG_2")]
    [InlineData("rte-1", "ROUTE_GEN")]              // "RTE 1" also names the legs list
    [InlineData("legs-1", "LEGS_PROGRESS")]         // "LEGS 1" also names DIR INTC
    [InlineData("perf-init-wgt", "PERF_INIT_WGT")]  // the four below draw no title at all
    [InlineData("perf-init-clb", "PERF_INIT_CLB")]
    [InlineData("perf-init-crz", "PERF_INIT_CRZ")]
    [InlineData("perf-init-des", "PERF_INIT_DES")]
    public void ResolvesCapturedPage(string fixture, string expected)
    {
        var page = Resolver.Resolve(CniFixtures.Load(fixture));

        Assert.NotNull(page);
        Assert.Equal(expected, page!.Name);
    }

    [Fact]
    public void EveryCapturedPageResolves()
    {
        var unresolved = CniFixtures.Names
            .Where(n => Resolver.Resolve(CniFixtures.Load(n)) is null)
            .ToList();

        Assert.Empty(unresolved);
    }

    /// <summary>
    /// "IFF" names three pages that differ only in their counter, so dropping the counter
    /// tie-break silently renders page 2 with page 1's layout.
    /// </summary>
    [Fact]
    public void CounterSeparatesPagesSharingATitle()
    {
        var iff = CniFixtures.Schema.Pages.Where(p => p.Title == "IFF").ToList();

        Assert.Equal(3, iff.Count);
        Assert.Equal(new[] { "1/3", "2/3", "3/3" }, iff.Select(p => p.Counter).Order());
    }

    /// <summary>
    /// The six CARP INIT pages share one title and none of them writes its counter out, so the
    /// literal tie-break cannot see them apart — and unlike the ARRIVAL families they are six
    /// different screens. Before the format was consulted this fell through to block count,
    /// which put page 2's data under whichever layout happened to be the nearest size.
    ///
    /// Pages 1, 2 and 5 are covered by captures above. Pages 3 and 4 were never visited, so
    /// they are driven with a counter and a title instead — enough to say they can be told
    /// apart, not that they render correctly.
    /// </summary>
    [Theory]
    [InlineData("3/5", "CARP_INIT_3")]
    [InlineData("4/5", "CARP_INIT_4")]
    public void CarpInitPagesAreSeparatedByTheirCounterFormat(string counter, string expected)
    {
        var data = CniFixtures.Load("dump-001") with
        {
            Title = "CARP 1 INIT",
            Blocks = new List<CniBlock> { new(1, null, counter, null) },
        };

        var page = Resolver.Resolve(data);

        Assert.NotNull(page);
        Assert.Equal(expected, page!.Name);
    }

    [Fact]
    public void UnknownTitleResolvesToNothing()
    {
        var data = CniFixtures.Load("dump-001") with { Title = "NO SUCH PAGE" };

        Assert.Null(Resolver.Resolve(data));
    }
}
