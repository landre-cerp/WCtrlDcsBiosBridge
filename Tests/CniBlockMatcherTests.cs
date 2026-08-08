using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class CniBlockMatcherTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    [Fact]
    public void FlattenKeepsDocumentOrder()
    {
        var data = CniFixtures.Load("dump-005");   // IFF, five nested toggles
        var flat = CniBlockMatcher.Flatten(data.Blocks);

        Assert.Equal(data.N, flat.Count);
        Assert.Equal(Enumerable.Range(1, flat.Count).ToArray(),
                     flat.Select(b => b.N ?? -1).ToArray());
    }

    [Theory]
    [MemberData(nameof(FixtureNames))]
    public void EveryBlockFindsASlot(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);

        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var matched = CniBlockMatcher.Align(blocks, page!.Slots);

        // Blanks excepted. The sim emits an element for every field of every leg whether or not
        // it holds anything, and on a route with a break the last leg sends one more of them
        // than the page has room for. A blank that finds no slot draws nothing, so it costs the
        // screen nothing — and pricing it apart from a real value is what stopped the landing
        // altitude going overboard in its place. A block carrying text still has to land.
        var orphans = blocks
            .Where((b, i) => matched[i] is null && !string.IsNullOrEmpty(b.V))
            .Select(b => $"n={b.N} v='{b.V}'")
            .ToList();

        Assert.True(orphans.Count == 0,
            $"{page.Name}: {orphans.Count}/{blocks.Count} blocs sans slot -> {string.Join(", ", orphans)}");
    }

    /// <summary>
    /// Literal text is what pins the alignment. If a block carrying "IDENT" is paired with a
    /// slot whose literal is something else, every position after it is suspect.
    /// </summary>
    [Theory]
    [MemberData(nameof(FixtureNames))]
    public void LiteralSlotsKeepTheirText(string fixture)
    {
        var data = CniFixtures.Load(fixture);
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);

        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var matched = CniBlockMatcher.Align(blocks, page!.Slots);

        for (var i = 0; i < blocks.Count; i++)
        {
            var slot = matched[i];
            if (slot?.Value is null) continue;

            Assert.Equal(slot.Value, blocks[i].V);
        }
    }

    /// <summary>
    /// A toggle's container arrives carrying children and no text of its own; pairing it with
    /// an ordinary text slot would push the words it wraps onto the wrong line.
    /// </summary>
    [Fact]
    public void ContainersPairWithContainers()
    {
        var data = CniFixtures.Load("232316-006");   // COMM TUNE U1
        var page = Resolver.Resolve(data);
        Assert.NotNull(page);

        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var matched = CniBlockMatcher.Align(blocks, page!.Slots);

        var withChildren = blocks.Where(b => b.C is { Count: > 0 }).ToList();
        Assert.NotEmpty(withChildren);

        for (var i = 0; i < blocks.Count; i++)
        {
            if (blocks[i].C is not { Count: > 0 }) continue;
            Assert.True(matched[i]?.IsContainer, $"bloc n={blocks[i].N} apparie hors conteneur");
        }
    }

    /// <summary>
    /// The schema lists every variant of every field while the sim sends only the visible
    /// ones, so a page that matched slot-for-slot would mean the schema had lost variants.
    /// </summary>
    [Fact]
    public void SchemaHoldsMoreSlotsThanTheSimSends()
    {
        var data = CniFixtures.Load("232316-006");
        var page = Resolver.Resolve(data)!;

        Assert.True(page.Slots.Count > data.N,
            $"slots={page.Slots.Count} blocs={data.N}");
    }

    public static TheoryData<string> FixtureNames()
    {
        var data = new TheoryData<string>();
        foreach (var name in CniFixtures.Names) data.Add(name);
        return data;
    }
}
