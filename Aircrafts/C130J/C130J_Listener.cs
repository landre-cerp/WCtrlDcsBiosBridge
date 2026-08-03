using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// C-130J CNI-MU repeater.
///
/// DCS-BIOS has no module for this aircraft, so nothing arrives on the DCS-BIOS stream while
/// it is loaded. Everything shown here comes from the wctrl-export.lua UDP feed, which scrapes
/// the pilot CNI's indication, and the CNI is therefore the only page this listener offers.
///
/// The indication carries element names and values and nothing else — no position, no font
/// size, no highlight. The layout comes from <c>Resources/c130j-cni-pages.json</c>, extracted
/// offline from the module's own page scripts by <c>tools/cni-schema</c>.
/// </summary>
internal sealed class C130J_Listener : AircraftListener
{
    private const string SchemaFile = "Resources/c130j-cni-pages.json";

    private readonly CniSchema? _schema;
    private readonly CniPageResolver? _resolver;
    private SimExportReceiver? _exportReceiver;

    /// <summary>
    /// Last page identified, so an unrecognised packet does not blank a screen that was
    /// showing something valid a moment ago.
    /// </summary>
    private CniPage? _page;

    /// <summary>
    /// What the crew's own switching has revealed about the rotaries, kept for the life of this
    /// listener. The GUIDs it is keyed on are regenerated every session, so it starts empty and
    /// is worth nothing to anyone else.
    /// </summary>
    private readonly CniSessionMap _session = new();

    /// <summary>
    /// The CNI is 25 characters across. It was squeezed into 24 for as long as that was
    /// thought to be the panel's grid, at the cost of every column of figures on the
    /// display; the panel takes 25 without complaint.
    /// </summary>
    protected override (int Lines, int Columns)? ScreenSize => (CniGrid.Lines, CniGrid.Columns);

    /// <summary>
    /// The module's own phosphor, which it names green_blue:
    /// <c>materials["green_blue"] = {5, 255, 63, 255}</c> in the C-130J's materials.lua,
    /// and <c>fonts["cni_font_green"]</c> is built from it.
    /// </summary>
    protected override string? DisplayGreenRgb => "05FF3F";

    public C130J_Listener(UserOptions options) : base(AircraftRegistry.C130J, options)
    {
        try
        {
            _schema = CniSchema.Load(Path.Combine(AppContext.BaseDirectory, SchemaFile));
            _resolver = new CniPageResolver(_schema);
            App.Logger.Info($"C-130J CNI schema loaded: {_schema.Pages.Count} pages");
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, $"Failed to load CNI schema: {SchemaFile}");
        }

        if (options.EnableLiveExport)
        {
            // Started before subscribing: the receiver lives as long as the process, so a
            // handler attached ahead of a throwing EnsureStarted would outlive this
            // half-built listener and keep being called on it.
            var receiver = SimExportReceiver.Shared;
            receiver.EnsureStarted();
            receiver.DataReceived += OnLiveExportData;
            _exportReceiver = receiver;
        }
    }

    protected override void RegisterCduControls() => RenderPlaceholder();

    // Nothing to register: DCS-BIOS exports no controls for the C-130J.
    protected override void RegisterFrontpanelControls() { }

    // Runs on the UDP receiver thread, like the F-14B(U) and A-10C live export paths.
    private void OnLiveExportData(SimExportData data)
    {
        // The export only sends a page when it changed, plus a heartbeat. A packet without
        // one carries no news, and clearing on it would make the display flicker.
        if (data.Cni is not { } cni || _resolver is null) return;

        var page = _resolver.Resolve(cni);
        if (page is null)
        {
            App.Logger.Debug($"CNI page not recognised: '{cni.Title}' ({cni.N} blocks)");
            return;
        }

        if (!ReferenceEquals(page, _page))
        {
            _page = page;
            App.Logger.Debug($"CNI page: {page.Name} ('{cni.Title}')");
        }

        var runs = CniGrid.Render(cni, page, _session);

        var c = GetCompositor(DEFAULT_PAGE);
        c.Clear();

        // Off by default, the compositor renders lowercase as small uppercase. The CNI draws
        // real mixed case on some pages, so ask for it — a fresh compositor each tick means
        // this has to be set every time.
        c.UseLowercaseFont();

        foreach (var run in runs)
        {
            if (run.Invert) c.Black().BGGreen();
            else c.Green().BGBlack();

            c.Line(run.Line)
             .Column(run.Column)
             .Small(run.Small)
             .Write(run.Text);
        }
    }

    private void RenderPlaceholder()
    {
        var c = GetCompositor(DEFAULT_PAGE);
        c.Clear();

        // Write() establishes column 0 before Centered so it knows the line width.
        if (_schema is null)
        {
            c.Line(4).Small().White().Write("").Centered("CNI SCHEMA MISSING");
            c.Line(5).Small().White().Write("").Centered("REINSTALL THE BRIDGE");
        }
        else if (_exportReceiver is null)
        {
            c.Line(4).Small().White().Write("").Centered("LIVE EXPORT DISABLED");
            c.Line(5).Small().White().Write("").Centered("ENABLE IT IN OPTIONS");
        }
        else
        {
            c.Line(4).Small().White().Write("").Centered("WAITING FOR CNI DATA");
            c.Line(5).Small().White().Write("").Centered("CHECK wctrl-export.lua");
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            // The receiver is shared across every listener that wants the feed (other CDUs
            // on the same or a different aircraft may still be reading it), so unsubscribe
            // rather than tearing the socket down.
            if (_exportReceiver != null)
                _exportReceiver.DataReceived -= OnLiveExportData;
            _exportReceiver = null;
        }
        base.Dispose(disposing);
    }
}
