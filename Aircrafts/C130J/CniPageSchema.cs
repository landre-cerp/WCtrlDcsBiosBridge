using System.Text.RegularExpressions;
using Newtonsoft.Json;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

internal enum CniAnchor
{
    Left = 0,
    Center = 1,
    Right = 2,
}

/// <summary>
/// One element of a CNI page as the module's own script builds it, in <c>Add()</c> order.
///
/// This is everything the indication does not carry. <c>list_indication</c> gives names and
/// values; where the text goes, how big it is and whether it is highlighted live only in the
/// page scripts, which <c>tools/cni-schema</c> replays offline to produce this.
/// </summary>
internal sealed class CniSlot
{
    /// <summary>Position in Add() order, 1-based. The join key against a runtime block.</summary>
    [JsonProperty("n")] public int N { get; init; }

    /// <summary>Display line 0..13, or null for an element with no grid position.</summary>
    [JsonProperty("l")] public int? Line { get; init; }

    /// <summary>Character offset from the display centre; negative is left of it.</summary>
    [JsonProperty("c")] public int? Col { get; init; }

    [JsonProperty("a")] public CniAnchor Anchor { get; init; }

    /// <summary>Literal text, when the element has one. These are the matcher's landmarks.</summary>
    [JsonProperty("v")] public string? Value { get; init; }

    /// <summary>
    /// Patterns the element's text can take, from the printf formats the page script gives it.
    /// What tells one dynamic field from its neighbours: a frequency fits "%d/%s" while the
    /// identifier next to it only fits "%s".
    /// </summary>
    [JsonProperty("f")] public List<string>? Formats { get; init; }

    /// <summary>
    /// How much <see cref="Formats"/> narrows things down, as literal characters outside the
    /// conversions. "%s" scores nothing; a format with punctuation scores more.
    /// </summary>
    [JsonProperty("w")] public int FormatWeight { get; init; }

    [JsonProperty("s")] public int Small { get; init; }

    /// <summary>Drawn with the inverted material.</summary>
    [JsonProperty("i")] public int Invert { get; init; }

    /// <summary>Container a toggle draws around its words; carries children at runtime.</summary>
    [JsonProperty("k")] public int Container { get; init; }

    /// <summary>Positioned outside the page grid, as STARTUP does. Never placed.</summary>
    [JsonProperty("off")] public int OffGrid { get; init; }

    /// <summary>1 for the title, 2 for the scratchpad — the only two the sim names.</summary>
    [JsonProperty("nm")] public int NamedRole { get; init; }

    /// <summary>
    /// The field this slot belongs to, taken from the module's controller name with its state
    /// tail removed. Elements sharing one are drawn from the same decision: on COMM TUNE U1 the
    /// GUARD word in the IDENT column and the GUARD toggle both run off <c>uhf1_guard</c>, so
    /// seeing either settles the other.
    /// </summary>
    [JsonProperty("ct")] public string? Controller { get; init; }

    /// <summary>1 on the slot the module draws when its field is the selected one.</summary>
    [JsonProperty("st")] public int State { get; init; }

    /// <summary>
    /// The controller behind an element the module builds in a single form. Deliberately apart
    /// from <see cref="Controller"/>, which only ever names a two-state field: this is read for
    /// nothing but tying a page to the radio it is tuned to, and letting it stand in for a field
    /// would have the IFF code readout deciding whether IDENT is lit beside it.
    /// </summary>
    [JsonProperty("cs")] public string? Source { get; init; }

    /// <summary>
    /// The column of a table this slot belongs to, or null when it is not in one.
    ///
    /// LANDING DATA is four rows of the same four flap settings, and one setting is selected for
    /// the page rather than for each row: the module names them <c>told_landing_flaps_100</c>,
    /// <c>told_landing_app_100</c>, <c>told_landing_thr_100</c>, all at the same grid column. So
    /// whatever settles one of them settles the column, which is what lets the header row — the
    /// only one made of literals — speak for the three rows of figures below it.
    /// </summary>
    public string? Column { get; private set; }

    internal void JoinColumn(string key) => Column = key;

    public bool IsStatic => Value != null;
    public bool IsSmall => Small != 0;
    public bool IsInvert => Invert != 0;
    public bool IsContainer => Container != 0;
    public bool IsSelected => State != 0;
    public bool IsPlaceable => OffGrid == 0 && Line is >= 0 && Col != null;

    /// <summary>
    /// The same element in its other state, where the module builds one. Null on an element
    /// that only exists in a single state — and that is what makes its arrival proof of the
    /// state, since the sim would not have drawn it otherwise.
    /// </summary>
    public CniSlot? Counterpart { get; private set; }

    /// <summary>
    /// Whether this slot's field is one the module builds in two states. Only those can be read
    /// as evidence of anything; a controller that drives a single element in a single form says
    /// nothing by turning up.
    /// </summary>
    public bool FieldHasTwoStates { get; private set; }

    internal void PairWith(CniSlot other)
    {
        Counterpart = other;
        other.Counterpart = this;
    }

    internal void MarkTwoState() => FieldHasTwoStates = true;

    private Regex[]? _compiled;

    /// <summary>True when the text could have come out of one of this element's formats.</summary>
    public bool FormatAccepts(string text)
    {
        if (Formats is not { Count: > 0 }) return false;

        _compiled ??= Formats
            .Select(f =>
            {
                try { return new Regex(f, RegexOptions.Compiled); }
                catch (ArgumentException) { return null; }
            })
            .Where(r => r is not null)
            .Select(r => r!)
            .ToArray();

        return _compiled.Any(r => r.IsMatch(text));
    }
}

/// <summary>
/// A rotary: the fields of one page that are mutually exclusive, exactly one of them selected.
///
/// The module names them off a common stem — power_up_align_gps, _last and _ref — but a stem is
/// not enough on its own, since uhf1_guard and uhf1_chan share one too. Members must also be
/// drawn together, which is what tells a rotary apart from two unrelated fields whose names
/// happen to rhyme.
/// </summary>
internal sealed record CniSelector(string Key, IReadOnlyList<string> Members);

internal sealed class CniPage
{
    [JsonProperty("id")] public int Id { get; init; }
    [JsonProperty("name")] public string Name { get; init; } = "";

    /// <summary>Literal title, when the page draws one.</summary>
    [JsonProperty("title")] public string? Title { get; init; }

    /// <summary>
    /// Patterns for the 25 pages that build their title from a printf format rather than a
    /// literal — "INAV%d CTRL SOLN", "%sLZ %2d INIT" and the like.
    /// </summary>
    [JsonProperty("rx")] public List<string>? TitlePatterns { get; init; }

    /// <summary>The "1/4"-style page counter, where the page has one. Breaks title ties.</summary>
    [JsonProperty("counter")] public string? Counter { get; init; }

    [JsonProperty("slots")] public List<CniSlot> Slots { get; init; } = new();

    /// <summary>
    /// Joins each element to itself in its other state.
    ///
    /// Position first, which is how a toggle's words pair: the OFF under one container and the
    /// OFF under the other sit in the same cell and differ only in size and highlight. What is
    /// left over then pairs on the line alone, for the fields whose two states are not the same
    /// width — ADF/MN/BTH writes "ADF/ " against the right margin when unselected and "ADF" two
    /// columns in when selected, so the pair never shares a column.
    /// </summary>
    internal void ResolveVariants()
    {
        var named = Slots.Where(s => s.Controller != null).ToList();

        PairWithin(named, s => (s.Controller, s.Line, s.Col, s.Anchor));
        PairWithin(named.Where(s => s.Counterpart is null), s => (s.Controller, s.Line));

        PairLeftovers();
        MarkTwoStateFields();
        Selectors = FindSelectors();
        FindColumns();
    }

    /// <summary>
    /// Ties together the fields of one table column.
    ///
    /// A column is what several rows call by the same last name at the same place: the four
    /// <c>_100</c> fields of LANDING DATA all sit at column 19, on four different lines, under
    /// four different row names. One flap setting is selected for the page, so those four are
    /// one decision — and the header row, alone in being made of literals, is the one the
    /// matcher can read it off.
    ///
    /// Two lines at least, and two row names at least: one field on its own is not a column, and
    /// neither are the two halves of a toggle, which share a line and differ in nothing but the
    /// last name this keys on.
    /// </summary>
    private void FindColumns()
    {
        var groups = Slots
            .Where(s => s is { FieldHasTwoStates: true, Controller: not null, Col: not null }
                        && s.Counterpart is not null)
            .GroupBy(s => $"{LastName(s.Controller!)}|{s.Col}|{(int)s.Anchor}", StringComparer.Ordinal);

        foreach (var group in groups)
        {
            var members = group.ToList();
            if (members.Select(s => s.Line).Distinct().Count() < 2) continue;
            if (members.Select(s => s.Controller).Distinct(StringComparer.Ordinal).Count() < 2) continue;

            foreach (var slot in members) slot.JoinColumn(group.Key);
        }
    }

    private static string LastName(string field)
    {
        var cut = field.LastIndexOf('_');
        return cut >= 0 ? field[(cut + 1)..] : field;
    }

    /// <summary>
    /// The rotaries on this page, three members or more.
    ///
    /// Two members are left out on purpose. Watching a rotary change is what says which of a
    /// member's two elements is the lit one — the members that did not move are the ones that
    /// were not selected either side of it — and with only two members every switch moves both,
    /// leaving nothing to reason from. Three is where it starts to work.
    /// </summary>
    public IReadOnlyList<CniSelector> Selectors { get; private set; } = Array.Empty<CniSelector>();

    private IReadOnlyList<CniSelector> FindSelectors()
    {
        var fields = Slots
            .Where(s => s.Controller != null && s.FieldHasTwoStates)
            .GroupBy(s => s.Controller!)
            .Where(g => g.Any(s => s.IsInvert))
            .ToDictionary(g => g.Key, g => g.ToList(), StringComparer.Ordinal);

        var found = new List<CniSelector>();

        foreach (var stem in fields.Keys.GroupBy(Stem).Where(g => g.Key.Length > 0))
        {
            var members = stem.OrderBy(m => m, StringComparer.Ordinal).ToList();
            if (members.Count < 3) continue;

            var lines = members.SelectMany(m => fields[m])
                               .Select(s => s.Line)
                               .Where(l => l is not null)
                               .Select(l => l!.Value)
                               .ToList();

            // Drawn together, within a couple of lines of one another. ADF/MN/BTH straddles two
            // because the module wraps it; anything wider is a stem shared by coincidence.
            if (lines.Count == 0 || lines.Max() - lines.Min() > 2) continue;

            found.Add(new CniSelector(stem.Key, members));
        }

        return found;
    }

    private static string Stem(string field)
    {
        var cut = field.LastIndexOf('_');
        return cut > 0 ? field[..cut] : "";
    }

    /// <summary>
    /// Pairs the highlighted and plain forms of everything the state tails did not already
    /// account for — the page counter among them, whose two forms share one controller and
    /// name no state at all. Nothing will ever settle which of those the sim drew, but pairing
    /// them is what stops the highlighted one being taken at face value.
    /// </summary>
    private void PairLeftovers()
    {
        var loose = Slots.Where(s => s.Counterpart is null && s.Line is not null).ToList();

        // Position and text only, controller deliberately left out: on INAV CTRL SENSORS the
        // two forms of INS1 are driven by different controllers, and keying on those would
        // leave each of them looking like the only one of its kind.
        PairWithin(loose, s => (s.Line, s.Col, s.Anchor, s.Value,
                                s.Formats is null ? "" : string.Join('|', s.Formats)),
                   opposed: static (a, b) => a.IsInvert != b.IsInvert);
    }

    /// <summary>
    /// Flags the slots whose field genuinely has two states.
    ///
    /// Only those can be read as evidence. A controller that never names a state drives one
    /// element in one form, so its arrival says nothing about anything — and taking it as proof
    /// is how MODE S came to sit on ON for a whole flight.
    /// </summary>
    private void MarkTwoStateFields()
    {
        foreach (var field in Slots.Where(s => s.Controller != null).GroupBy(s => s.Controller))
        {
            if (field.Select(s => s.IsSelected).Distinct().Count() < 2) continue;
            foreach (var slot in field) slot.MarkTwoState();
        }
    }

    /// <summary>
    /// Pairs opposite states inside each bucket, in schema order. The extractor emits the two
    /// variants of a field next to one another, so consecutive is the right reading and a
    /// bucket that holds an odd one out simply leaves it unpaired.
    /// </summary>
    private static void PairWithin<TKey>(IEnumerable<CniSlot> slots, Func<CniSlot, TKey> key,
                                         Func<CniSlot, CniSlot, bool>? opposed = null)
    {
        opposed ??= static (a, b) => a.IsSelected != b.IsSelected;

        foreach (var bucket in slots.GroupBy(key))
        {
            CniSlot? pending = null;
            foreach (var slot in bucket)
            {
                if (pending is not null && opposed(pending, slot))
                {
                    pending.PairWith(slot);
                    pending = null;
                }
                else
                {
                    pending = slot;
                }
            }
        }
    }
}

internal sealed class CniSchema
{
    [JsonProperty("columns")] public int Columns { get; init; } = CniGrid.Columns;
    [JsonProperty("lines")] public int Lines { get; init; } = 14;
    [JsonProperty("pages")] public List<CniPage> Pages { get; init; } = new();

    public static CniSchema Load(string path)
    {
        var json = File.ReadAllText(path);
        var schema = JsonConvert.DeserializeObject<CniSchema>(json)
                     ?? throw new InvalidDataException($"CNI schema is empty: {path}");

        foreach (var page in schema.Pages)
            page.ResolveVariants();

        return schema;
    }
}
