using WCtrlDcsBiosBridge.Services;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.F14;

/// <summary>
/// F-14B(U): an F-14B plus the CDNU.
///
/// Every F-14 control comes from the base listener. DCS-BIOS only exports them once
/// its F-14 module lists "F-14BU" among its aircraft names — a local edit, since that
/// variant is not supported upstream. The CDNU is not carried by DCS-BIOS at all and
/// arrives over the wctrl-export.lua UDP feed instead.
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
    /// The CDNU font maps low control codes onto symbols. They survive the export intact
    /// (JSON escapes them as \u00XX), so they arrive here as-is and need translating to
    /// glyphs the CDU font actually carries.
    ///
    /// Established from captured traffic: U+000E sits inside coordinates and headings
    /// ("N31.33.1", "226.M"); U+000F and U+0010 appear only ever at column 0 and column 21,
    /// the two line-select edges; U+0016 flanks the entry it scrolls to, as in ".to. Flt Pln".
    ///
    /// U+0012 sits at column 0 of the scratchpad row only, and is a double-headed vertical
    /// arrow. The font has one, but keyed as Delta: a glyph's Character is only the device
    /// slot it loads into, and the bitmap in that slot decides what is actually drawn.
    /// </summary>
    private static readonly Dictionary<char, char> CdnuGlyphs = new()
    {
        ['\u000E'] = '\u00B0',   // degree
        ['\u000F'] = '\u2190',   // left line-select marker
        ['\u0010'] = '\u2192',   // right line-select marker
        ['\u0016'] = '\u2193',   // down arrow, flanking the entry it scrolls to
        ['\u0012'] = '\u0394',   // the Delta slot draws a double-headed vertical arrow
        ['_'] = '\u2B21',   // scratchpad cursor: the underscore slot draws a cross, not a block
    };

    private readonly Key _cdnuDisplayKey;

    private SimExportReceiver? _exportReceiver;

    public F14BU_Listener(UserOptions options) : base(AircraftRegistry.F14BU, options)
    {
        _cdnuDisplayKey = Enum.TryParse<Key>(options.F14.CdnuKey, out var cdnuKey)
            ? cdnuKey : Key.Data;

        AddNewPage(CDNU_PAGE);

        if (options.EnableLiveExport)
        {
            _exportReceiver = new SimExportReceiver();
            _exportReceiver.DataReceived += OnLiveExportData;
            _exportReceiver.Start();
        }
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

        RenderPlaceholder();
        _currentPage = CDNU_PAGE;
    }

    // Runs on the UDP receiver thread, like the A-10C live export path.
    private void OnLiveExportData(SimExportData data)
    {
        if (data.Cdnu == null || data.Cdnu.Count == 0)
            return;

        var c = GetCompositor(CDNU_PAGE);
        c.Clear();

        // Off by default, the compositor renders lowercase as small uppercase. The CDNU
        // font carries real lowercase, so ask for it — a fresh compositor each tick means
        // this has to be set every time.
        c.UseLowercaseFont();

        for (int row = 0; row < CDNU_LINE_COUNT && row < data.Cdnu.Count; row++)
            c.Green()
             .Line(CDNU_FIRST_LINE + row)
             .Column(CDNU_FIRST_COLUMN)
             .Write(MapGlyphs(data.Cdnu[row]));
    }

    /// <summary>
    /// Translates the CDNU's control-code symbols to the CDU font's glyphs. Anything the
    /// table does not cover is blanked rather than passed through: an unmapped code has no
    /// glyph and would otherwise render as a hole in the line.
    ///
    /// Case is preserved: the CDNU labels it mixed ("Bagram Departure", "Flt Pln"), and
    /// f14bu-font-21x31.json carries the lowercase bitmaps the shared A-10C font lacks.
    /// </summary>
    private static string MapGlyphs(string? raw)
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
        if (_exportReceiver == null)
        {
            c.Line(4).Small().White().Write("").Centered("LIVE EXPORT DISABLED");
            c.Line(5).Small().White().Write("").Centered("ENABLE IT IN OPTIONS");
        }
        else
        {
            c.Line(4).Small().White().Write("").Centered("WAITING FOR CDNU DATA");
            c.Line(5).Small().White().Write("").Centered("CHECK wctrl-export.lua");
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _exportReceiver?.Dispose();
            _exportReceiver = null;
        }
        base.Dispose(disposing);
    }
}
