using Newtonsoft.Json;
using WCtrlDcsBiosBridge.Aircrafts.C130J;
using WwDevicesDotNet;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Holds the text the CNI can print against the font the panel is given.
///
/// A character with no glyph is not reported anywhere: the upload skips it, the panel draws
/// whatever it already held, and the only way anyone finds out is by reading a page on the
/// device and noticing. It took a photograph to catch the '&amp;' of "&lt;WT &amp; BAL" — one
/// occurrence in 145 pages — and the em dash that rules the alternate-route pages had been
/// missing twenty times over without anyone saying so.
///
/// Only the literals can be checked. What a dynamic field will hold is not knowable here, but
/// the module writes those with the same font it writes its labels with, so a page whose labels
/// are covered is a page whose values almost certainly are.
///
/// This is the check that would have found all three at once, and it is the one that will find
/// the next: a module update adding a page adds its labels, and they arrive here first.
/// </summary>
public class CniFontCoverageTests
{
    private static McduFontFile Font(string file)
    {
        var path = Path.Combine(AppContext.BaseDirectory, "Resources", file);
        var font = JsonConvert.DeserializeObject<McduFontFile>(File.ReadAllText(path));

        Assert.NotNull(font);
        return font!;
    }

    public static TheoryData<string> FontFiles() =>
        new() { "c130j-font-21x31.json", "c130j-font-21x32.json" };

    [Theory]
    [MemberData(nameof(FontFiles))]
    public void EveryCharacterThePagesPrintHasAGlyph(string file)
    {
        var font = Font(file);

        var large = font.LargeGlyphs.Select(g => g.Character).ToHashSet();
        var small = font.SmallGlyphs.Select(g => g.Character).ToHashSet();

        // Both sizes, because the CNI uses them side by side: 44% of its placeable slots are
        // small, and a glyph drawn in one set and not the other fails only on the pages that
        // happen to ask for the size it lacks.
        var gaps = new SortedDictionary<char, string>();

        foreach (var page in CniFixtures.Schema.Pages)
        {
            foreach (var slot in page.Slots)
            {
                if (slot.Value is not { Length: > 0 } literal) continue;

                // Through the renderer's own substitutions. The module writes a degree sign as
                // '^' and an empty field as ']', and neither is expected in the font under the
                // character the page script spells it with.
                foreach (var ch in CniGrid.MapGlyphs(literal))
                {
                    if (large.Contains(ch) && small.Contains(ch)) continue;

                    gaps.TryAdd(ch, $"'{ch}' (U+{(int)ch:X4}) — {page.Name}: \"{literal}\"");
                }
            }
        }

        Assert.True(gaps.Count == 0,
            $"{file}: {gaps.Count} caractere(s) sans glyphe -> {string.Join(" | ", gaps.Values)}");
    }
}
