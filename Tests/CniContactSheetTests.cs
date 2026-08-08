using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Renders every captured page to a text file beside the test binary.
///
/// Layout cannot be proved without holding it against the cockpit, and doing that page by
/// page does not scale. This turns the check into reading one sheet: every page the probe
/// captured, drawn exactly as the CDU would draw it, side by side with what the sim sent.
/// </summary>
public class CniContactSheetTests
{
    private static readonly CniPageResolver Resolver = new(CniFixtures.Schema);

    [Fact]
    public void WriteContactSheet()
    {
        var path = Path.Combine(AppContext.BaseDirectory, "cni-contact-sheet.txt");
        using var sheet = new StreamWriter(path, append: false);

        foreach (var name in CniFixtures.Names)
        {
            var data = CniFixtures.Load(name);
            var page = Resolver.Resolve(data);

            sheet.WriteLine(new string('=', 60));
            sheet.WriteLine($"{name}  titre={data.Title ?? ""}  blocs={data.N}  " +
                            $"page={page?.Name ?? "NON RECONNUE"}");
            sheet.WriteLine(new string('=', 60));

            if (page is null)
            {
                sheet.WriteLine("(non rendue)");
                sheet.WriteLine();
                continue;
            }

            var runs = CniGrid.Render(data, page);
            var lines = CniGrid.ToLines(runs);

            sheet.WriteLine("       +" + new string('-', CniGrid.Columns) + "+");
            for (var i = 0; i < lines.Length; i++)
                sheet.WriteLine($"  L{i,2}  |{lines[i]}|");
            sheet.WriteLine("       +" + new string('-', CniGrid.Columns) + "+");

            // Small text is marked so the sheet shows where the size distinction landed.
            var smallRuns = runs.Where(r => r.Small).Select(r => $"L{r.Line}:{r.Text.Trim()}");
            sheet.WriteLine("  petite police: " + string.Join(" | ", smallRuns));

            var invertRuns = runs.Where(r => r.Invert).Select(r => $"L{r.Line}:{r.Text.Trim()}");
            sheet.WriteLine("  inverse: " + string.Join(" | ", invertRuns));

            // Where the matcher put each block. The sheet shows what came out; this shows the
            // decision behind it, which is the only way to tell a layout fault from a matching
            // one — LANDING DATA looked like a placement bug and was an alignment drifting
            // across rows.
            var blocks = CniBlockMatcher.Flatten(data.Blocks);
            var matched = CniBlockMatcher.Align(blocks, page.Slots);

            sheet.WriteLine("  places: " + string.Join(" ", blocks.Select((b, i) =>
            {
                var slot = matched[i];
                return slot is null
                    ? $"[{b.V}]->rien"
                    : $"[{b.V}]->n{slot.N}L{slot.Line}C{slot.Col}{(slot.IsInvert ? "i" : "")}";
            })));

            sheet.WriteLine("  recu: " + string.Join(" ",
                CniBlockMatcher.Flatten(data.Blocks)
                    .Select(b => b.V)
                    .Where(v => !string.IsNullOrWhiteSpace(v))
                    .Select(v => $"[{v}]")));
            sheet.WriteLine();
        }

        Assert.True(File.Exists(path));
    }
}
