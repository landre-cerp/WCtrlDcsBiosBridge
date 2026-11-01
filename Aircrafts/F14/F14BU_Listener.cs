using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.F14;

/// <summary>
/// F-14B(U): an F-14B plus the CDNU.
///
/// Everything comes off DCS-BIOS. The F-14 controls arrive through the base listener — the
/// F-14 module lists "F-14BU" among its aircraft names as of v0.11.7 — and the CDNU through
/// RIO_CDNU_LINE1..8, which that module has carried since 2026-09-08. No release has those
/// eight yet, so the CDNU needs a nightly build dated 2026-09-11 or later; without them this
/// listener still gives the aircraft, and says on the page why the CDNU is empty.
/// </summary>
internal sealed class F14BU_Listener : F14_Listener
{
    private const string CDNU_PAGE = "CDNU";
    private const int CDNU_LINE_COUNT = 8;

    /// <summary>
    /// Zero-based screen row the first CDNU row lands on (the display's third line).
    /// The eight rows are offset so each one sits level with a line-select key
    /// instead of starting at the top of the display.
    /// </summary>
    private const int CDNU_FIRST_LINE = 2;

    /// <summary>
    /// Column the CDNU rows start at. The rows are 22 characters against the CDU's 24,
    /// so shifting by one centres them instead of leaving the slack on the right.
    /// </summary>
    private const int CDNU_FIRST_COLUMN = 1;

    /// <summary>
    /// The CDNU draws its symbols on low control codes, and the DCS-BIOS module swaps each one
    /// for a printable Latin-1 stand-in before exporting the row (its cdnu_replace_map). Those
    /// reach us intact — the string listener decodes a byte per character as ISO-8859-1 — and
    /// have to be turned into glyphs the CDU font actually carries:
    ///
    ///   « » are the line-select markers, { } the arrows flanking the entry they scroll to,
    ///   ® the scratchpad's double-headed vertical arrow, © its horizontal one.
    ///
    /// Safe as a straight substitution: none of those six are on the CDNU's own keyboard, so a
    /// stand-in can only be a stand-in. The degree sign is the exception that needs no entry —
    /// the module emits a real U+00B0 and the font has that slot.
    ///
    /// U+0013, the scratchpad's diamond, is *not* in the module's table and arrives raw.
    ///
    /// A glyph's Character is only the device slot it loads into — the bitmap in that slot
    /// decides what is drawn, and the stock font is full of mismatches. The double-headed
    /// vertical arrow lives under Delta, the block the cursor needs lives under the hexagon,
    /// and the underscore slot draws a cross. The diamond had no equivalent at all, so it is
    /// drawn into the device's spare U+25A1 slot by the font generator.
    ///
    /// The five arrows are drawn by tools/f14bu-font/build-arrows.py rather than inherited from
    /// the A-10C, whose heads are wide enough that a left arrow and a right one are hard to tell
    /// apart at this size. The horizontal double-headed one cannot be filed under U+2194 at all:
    /// a glyph only reaches the panel when the device's packet map lists its character, and none
    /// of the 110 slots it exposes is U+2194. So it sits in the spare U+25C0 slot, a
    /// left-pointing triangle in name only.
    /// </summary>
    private static readonly Dictionary<char, char> CdnuGlyphs = new()
    {
        ['\u00AB'] = '\u2190',   // « line-select marker, left
        ['\u00BB'] = '\u2192',   // » line-select marker, right
        ['{'] = '\u2191',        // up arrow, flanking the entry it scrolls to
        ['}'] = '\u2193',        // down arrow, flanking the entry it scrolls to
        ['\u00AE'] = '\u0394',   // ® scratchpad: double-headed vertical arrow
        ['\u00A9'] = '\u25C0',   // © scratchpad: double-headed horizontal arrow
        ['\u0013'] = '\u25A1',   // scratchpad: diamond with a centre pip
        ['_'] = '\u2B21',        // scratchpad cursor: the underscore slot draws a cross
    };

    private readonly Key _cdnuDisplayKey;

    /// <summary>The eight rows as they last arrived, untranslated.</summary>
    private readonly string[] _cdnuRows = new string[CDNU_LINE_COUNT];

    /// <summary>Whether the installed DCS-BIOS declares the eight CDNU rows.</summary>
    private bool _cdnuAvailable;

    public F14BU_Listener(UserOptions options) : base(AircraftRegistry.F14BU, options)
    {
        _cdnuDisplayKey = Enum.TryParse<Key>(options.F14.CdnuKey, out var cdnuKey)
            ? cdnuKey : Key.Data;

        AddNewPage(CDNU_PAGE);
    }

    protected override void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == _cdnuDisplayKey)
        {
            _currentPage = CDNU_PAGE;
            return;
        }

        base.HandleKeyDown(sender, e);
    }

    protected override void RegisterCduControls()
    {
        base.RegisterCduControls();

        RegisterCdnuControls();
        RenderPlaceholder();

        // The CDNU is what the variant is for, so it is the page the aircraft opens on —
        // the RIO and radio pages keep their own keys.
        _currentPage = CDNU_PAGE;
    }

    /// <summary>
    /// Registers the eight rows, tolerating a DCS-BIOS that predates them. The resolver throws
    /// on a control the installed module does not declare, and an F-14B(U) with no CDNU page is
    /// still an F-14B(U) — losing the RIO and radio pages and the gear lights along with it
    /// would be the worse trade.
    /// </summary>
    private void RegisterCdnuControls()
    {
        try
        {
            for (var row = 0; row < CDNU_LINE_COUNT; row++)
            {
                var slot = row;
                RegisterStr($"RIO_CDNU_LINE{slot + 1}", s =>
                {
                    _cdnuRows[slot] = s;
                    RenderCdnuPage();
                });
            }

            _cdnuAvailable = true;
        }
        catch (Exception ex)
        {
            _cdnuAvailable = false;
            App.Logger.Warn(ex, "F-14B(U) CDNU page disabled: this DCS-BIOS does not declare " +
                                "RIO_CDNU_LINE1..8. A nightly dated 2026-09-11 or later does.");
        }
    }

    private void RenderCdnuPage()
    {
        // Composed off-screen and copied over in one go. Clearing the live page first leaves it
        // blank until the eight rows are rewritten, and the display tick can catch it like that —
        // which shows as flicker, every row update.
        var page = pages[CDNU_PAGE];
        var scratch = new Screen(page.LineCount, page.ColumnCount);
        var c = new Compositor(scratch);

        // Off by default, the compositor renders lowercase as small uppercase. The CDNU
        // font carries real lowercase, so ask for it — a fresh compositor each time means
        // this has to be set every time. The RIO and radio pages compose through their own
        // compositors and keep the small uppercase their labels are written for.
        c.UseLowercaseFont();

        for (var row = 0; row < CDNU_LINE_COUNT; row++)
            c.Green()
             .Line(CDNU_FIRST_LINE + row)
             .Column(CDNU_FIRST_COLUMN)
             .Write(MapGlyphs(_cdnuRows[row]));

        page.CopyFrom(scratch);
    }

    /// <summary>
    /// Translates the CDNU's stand-in characters to the CDU font's glyphs. A control code the
    /// table does not cover is blanked rather than passed through: it has no glyph and would
    /// otherwise render as a hole in the line.
    ///
    /// Case is preserved: the CDNU labels it mixed ("Bagram Departure", "Flt Pln"), and
    /// f14bu-font-21x31.json carries the lowercase bitmaps the shared A-10C font lacks.
    /// </summary>
    internal static string MapGlyphs(string? raw)
    {
        if (string.IsNullOrEmpty(raw)) return string.Empty;

        return string.Create(raw.Length, raw, static (dst, src) =>
        {
            for (int i = 0; i < src.Length; i++)
            {
                char ch = src[i];
                if (CdnuGlyphs.TryGetValue(ch, out var glyph)) dst[i] = glyph;
                else if (ch < ' ') dst[i] = ' ';
                else dst[i] = ch;
            }
        });
    }

    private void RenderPlaceholder()
    {
        var c = GetCompositor(CDNU_PAGE);
        c.Clear();

        // Write() establishes column 0 before Centered so it knows the line width.
        if (_cdnuAvailable)
        {
            c.Line(4).Small().White().Write("").Centered("WAITING FOR CDNU DATA");
        }
        else
        {
            c.Line(4).Small().White().Write("").Centered("CDNU NOT IN DCS-BIOS");
            c.Line(5).Small().White().Write("").Centered("NIGHTLY 2026-09-11 OR UP");
        }
    }
}
