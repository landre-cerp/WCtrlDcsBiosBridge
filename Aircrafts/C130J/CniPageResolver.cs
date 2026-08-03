using System.Text.RegularExpressions;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Works out which of the 145 CNI pages is on screen.
///
/// The sim publishes no page identifier, so this reads the title the page draws. That is a
/// heuristic and is treated as one: titles repeat across pages, 25 of them are built from a
/// format rather than a literal, and one page draws no title at all. Ties are broken on the
/// page counter, then on how closely the block count fits, and an unrecognised page returns
/// null rather than a guess — a page rendered with the wrong layout looks like it works.
/// </summary>
internal sealed class CniPageResolver
{
    private static readonly Regex CounterPattern = new(@"^\s*\d+\s*/\s*\d+\s*$", RegexOptions.Compiled);

    private readonly Dictionary<string, List<CniPage>> _byTitle;
    private readonly List<(Regex Pattern, CniPage Page)> _byPattern;
    private readonly List<CniPage> _untitled;

    public CniPageResolver(CniSchema schema)
    {
        _byTitle = new Dictionary<string, List<CniPage>>(StringComparer.Ordinal);
        _byPattern = new List<(Regex, CniPage)>();
        _untitled = new List<CniPage>();

        foreach (var page in schema.Pages)
        {
            if (!string.IsNullOrEmpty(page.Title))
            {
                if (!_byTitle.TryGetValue(page.Title, out var list))
                    _byTitle[page.Title] = list = new List<CniPage>();
                list.Add(page);
            }
            else if (page.TitlePatterns is { Count: > 0 })
            {
                foreach (var rx in page.TitlePatterns)
                {
                    try
                    {
                        _byPattern.Add((new Regex(rx, RegexOptions.Compiled), page));
                    }
                    catch (ArgumentException)
                    {
                        // A pattern the extractor could not express cleanly costs that page its
                        // title route; the untitled fallback still covers it.
                        _untitled.Add(page);
                    }
                }
            }
            else
            {
                _untitled.Add(page);
            }
        }
    }

    public CniPage? Resolve(CniData data)
    {
        var blocks = CniBlockMatcher.Flatten(data.Blocks);
        var title = data.Title ?? "";

        // Shape alone decides only for a page that genuinely draws no title. A title that is
        // present but unrecognised means the schema is out of date or the page is new, and
        // falling back on block count there would dress it in some other page's layout.
        if (title.Length == 0)
            return ResolveUntitled(blocks);

        var candidates = Candidates(title);
        if (candidates.Count == 0)
            return null;

        if (candidates.Count == 1)
            return candidates[0];

        var counter = blocks
            .Select(b => b.V)
            .FirstOrDefault(v => !string.IsNullOrEmpty(v) && CounterPattern.IsMatch(v));

        if (counter != null)
        {
            var normalised = Normalise(counter);
            var onCounter = candidates
                .Where(p => p.Counter != null && Normalise(p.Counter) == normalised)
                .ToList();

            if (onCounter.Count == 1) return onCounter[0];
            if (onCounter.Count > 1) candidates = onCounter;

            // A page whose counter is drawn from a controller rather than written out has no
            // literal to compare, so the six CARP INIT pages all arrive here indistinguishable
            // - and they are six different screens, not one screen under six names. The page
            // number survives in the format its counter slot carries: CARP INIT 2 declares
            // "^2/%d$" and only it can produce "2/5". Cheaper and surer than the block count,
            // because it is the page saying which page it is.
            if (onCounter.Count == 0)
            {
                var onFormat = candidates.Where(p => CounterFormatFits(p, counter)).ToList();

                if (onFormat.Count == 1) return onFormat[0];
                if (onFormat.Count > 1) candidates = onFormat;
            }
        }

        // Still tied, so ask which page's landmarks the sim actually drew. Size was the tie-
        // break before and it is a poor judge: RTE 1 sent 32 blocks, which put it nearer the
        // 27 slots of the legs list than the 42 of the general page, and it is the general
        // page - 28 of whose 29 static strings were on screen against 5 of the legs list's 9.
        // Counting them costs one pass and answers the question that was being guessed at.
        //
        // Size stays as the last word, for the families where it genuinely does not matter:
        // ARRIVAL RWYS, ARRIVAL STARS and STAR REVIEW hold pages laid out identically down to
        // the column, differing only in which controller feeds them, which rendering never
        // reads. Any of them draws the same screen.
        return candidates
            .OrderByDescending(p => LandmarkScore(p, blocks))
            .ThenBy(p => Math.Abs(p.Slots.Count - blocks.Count))
            .ThenBy(p => p.Id)
            .First();
    }

    private List<CniPage> Candidates(string title)
    {
        if (title.Length > 0 && _byTitle.TryGetValue(title, out var exact))
            return exact;

        if (title.Length == 0)
            return new List<CniPage>();

        return _byPattern
            .Where(e => e.Pattern.IsMatch(title))
            .Select(e => e.Page)
            .Distinct()
            .ToList();
    }

    /// <summary>
    /// How many of a page's static strings the sim actually drew.
    ///
    /// The strongest evidence available short of a page identifier, and the sim publishes none.
    /// A page's literals are written into its script and cannot vary, so a page that is on
    /// screen shows nearly all of them and a page that is not shows the handful it happens to
    /// share. Distinct values, because a page repeating "/" nine times is not nine matches.
    /// </summary>
    private static int LandmarkScore(CniPage page, IReadOnlyList<CniBlock> blocks)
    {
        var drawn = blocks
            .Select(b => b.V)
            .Where(v => !string.IsNullOrEmpty(v))
            .ToHashSet(StringComparer.Ordinal);

        return page.Slots
            .Select(s => s.Value)
            .Where(v => !string.IsNullOrEmpty(v))
            .Distinct(StringComparer.Ordinal)
            .Count(v => drawn.Contains(v!));
    }

    /// <summary>
    /// A page that draws no title — BASIC, the four PERF INIT pages and eight others. Shape
    /// alone was deciding between twelve pages whose slot counts run from 1 to 52, which is
    /// close to picking by size and nothing else, so the landmarks lead here too. Size is kept
    /// as the tie-break and as the guard: with nothing recognised and a shape that fits no
    /// page, this returns null rather than dressing the screen in the nearest layout.
    /// </summary>
    private CniPage? ResolveUntitled(IReadOnlyList<CniBlock> blocks)
    {
        CniPage? best = null;
        var bestScore = -1;
        var bestDelta = int.MaxValue;

        foreach (var page in _untitled)
        {
            var score = LandmarkScore(page, blocks);
            var delta = Math.Abs(page.Slots.Count - blocks.Count);

            if (score > bestScore || (score == bestScore && delta < bestDelta))
            {
                best = page;
                bestScore = score;
                bestDelta = delta;
            }
        }

        return bestScore > 0 || bestDelta <= 4 ? best : null;
    }

    /// <summary>
    /// Whether this page has a slot whose declared format the observed counter fits, and which
    /// is a counter's format rather than any field that happens to match. Only formats naming
    /// the page number qualify — a bare "%d/%d" would fit every page in the family and settle
    /// nothing, so it is left to the tie-break below.
    /// </summary>
    private static bool CounterFormatFits(CniPage page, string counter)
    {
        var normalised = Normalise(counter);

        foreach (var slot in page.Slots)
        {
            if (slot.Formats is null) continue;

            foreach (var format in slot.Formats)
            {
                if (!CounterFormat.IsMatch(format)) continue;

                try
                {
                    if (Regex.IsMatch(normalised, format)) return true;
                }
                catch (ArgumentException)
                {
                    // Same tolerance as the title patterns: a format the extractor could not
                    // express is skipped rather than fatal.
                }
            }
        }

        return false;
    }

    /// <summary>
    /// A format that pins the page number: anchored, then a literal digit and a slash before
    /// any conversion. The optional group is the alternation the extractor wraps every pattern
    /// in so a field drawn entirely as placeholder still matches.
    /// </summary>
    private static readonly Regex CounterFormat = new(@"^\^(?:\(\?:)?\d+/", RegexOptions.Compiled);

    private static string Normalise(string counter) =>
        counter.Replace(" ", "").Replace("\t", "");
}
