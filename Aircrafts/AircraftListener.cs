using ClassLibraryCommon;
using System.Collections.Concurrent;
using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Interfaces;
using DCS_BIOS.Serialized;
using Newtonsoft.Json;
using System.IO;
using Timer = System.Timers.Timer;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Devices.Cdu;
using WCtrlDcsBiosBridge.Devices.Frontpanels;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal abstract class AircraftListener : IDcsBiosListener, IDisposable
{
    private static readonly double _TICK_DISPLAY = 100;

    // The annunciators run at 30 Hz on their own timer while the screen keeps its 10 Hz.
    // DCS-BIOS exports every rendered frame, so the tick is the only thing quantizing a lamp,
    // and a flashing caution sampled at 10 Hz blinks visibly slower than the cockpit's. A LED
    // tick costs nothing unless something moved (see CduRenderer.RenderLeds), a screen tick
    // copies the whole buffer.
    private static readonly double _TICK_LEDS = 33;

    private readonly Timer _DisplayCDUTimer;
    private readonly Timer _LedCDUTimer;
    protected AircraftCduContext? cdu;

    private bool _disposed;

    private readonly DCSBIOSOutput _UpdateCounterDCSBIOSOutput;
    private readonly object _UpdateCounterLockObject = new();
    private bool _HasSyncOnce;
    private uint _Count;

    protected readonly UserOptions options;
    protected readonly AircraftDescriptor descriptor;

    protected const string DEFAULT_PAGE = "default";
    protected string _currentPage = DEFAULT_PAGE;

    /// <summary>
    /// Semantic flight deck state populated by this listener. Frontpanel renderers
    /// read from it; the listener never touches frontpanel device types.
    /// </summary>
    public FlightDeckState FlightDeck { get; } = new();

    // adresse DCS-BIOS -> handlers
    private readonly Dictionary<uint, List<Action<uint>>> _dataHandlers = new();
    private readonly Dictionary<uint, List<Action<string>>> _stringHandlers = new();

    private void RegisterCore(DCSBIOSOutput output, Action<uint> handler)
    {
        if (!_dataHandlers.TryGetValue(output.Address, out var list))
            _dataHandlers[output.Address] = list = new();
        list.Add(data => handler(output.GetUIntValue(data)));
    }

    private void RegisterStringCore(DCSBIOSOutput output, Action<string> handler)
    {
        if (!_stringHandlers.TryGetValue(output.Address, out var list))
            _stringHandlers[output.Address] = list = new();
        list.Add(handler);
    }

    // Use for bitfield registers when named DCS-BIOS outputs are unavailable or have incorrect
    // mask/shift definitions. The value passed to the handler is the raw unmasked 16-bit register.
    // NOTE: DCSBIOSProtocolParser only dispatches events for addresses on its broadcast whitelist.
    // Named outputs get whitelisted automatically via DCSBIOSOutput.Address setter.
    // RegisterRaw bypasses that path and whitelists the address explicitly via a throw-away output.
    protected void RegisterRaw(uint address, Action<uint> handler)
    {
        _ = new DCSBIOSOutput { Address = address }; // side effect: whitelists address with protocol parser
        if (!_dataHandlers.TryGetValue(address, out var list))
            _dataHandlers[address] = list = new();
        list.Add(handler);
    }

    protected void RegisterUInt(string controlId, Action<uint> handler)
    {
        var ctrl = DCSBIOSControlLocator.GetUIntDCSBIOSOutput(controlId);
        if (ctrl is null) return;
        RegisterCore(ctrl, handler);
    }

    protected void RegisterUInt(string controlId, Action<DCSBIOSOutput, uint> handler)
    {
        var ctrl = DCSBIOSControlLocator.GetUIntDCSBIOSOutput(controlId);
        if (ctrl is null) return;
        RegisterCore(ctrl, v => handler(ctrl, v));
    }

    protected void RegisterUInt(DCSBIOSOutput? output, Action<uint> handler)
    {
        if (output is null) return;
        RegisterCore(output, handler);
    }

    protected void RegisterStr(string controlId, Action<string> handler)
    {
        var ctrl = DCSBIOSControlLocator.GetStringDCSBIOSOutput(controlId);
        if (ctrl is null) return;
        RegisterStringCore(ctrl, handler);
    }

    protected void RegisterStr(DCSBIOSOutput? output, Action<string> handler)
    {
        if (output is null) return;
        RegisterStringCore(output, handler);
    }

    protected void RegisterLight(string controlId, Action<uint> handler)
    {
        if (options.DisableLightingManagement) return;
        RegisterUInt(controlId, handler);
    }

    protected void RegisterLight(string controlId, Action<DCSBIOSOutput, uint> handler)
    {
        if (options.DisableLightingManagement) return;
        RegisterUInt(controlId, handler);
    }

    protected void RegisterLight(DCSBIOSOutput? output, Action<uint> handler)
    {
        if (options.DisableLightingManagement) return;
        RegisterUInt(output, handler);
    }

    protected DCSBIOSOutput? ResolveUInt(string controlId)
        => DCSBIOSControlLocator.GetUIntDCSBIOSOutput(controlId);

    protected DCSBIOSOutput? ResolveStr(string controlId)
        => DCSBIOSControlLocator.GetStringDCSBIOSOutput(controlId);

    // Dispatch unique : implémentation explicite des interfaces pour que les
    // classes filles ne puissent plus l'override. Elles passent par RegisterUInt/RegisterStr.
    void IDcsBiosDataListener.DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);
            if (_dataHandlers.TryGetValue(e.Address, out var handlers))
                foreach (var h in handlers) h(e.Data);
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS data");
        }
    }

    void IDCSBIOSStringListener.DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        try
        {
            if (_stringHandlers.TryGetValue(e.Address, out var handlers))
                foreach (var h in handlers) h(e.StringData);
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS string data");
        }
    }


    // Concurrent because the display timer reads pages[_currentPage] on a
    // thread-pool thread while DCS-BIOS handlers may add a page via
    // GetCompositor/AddNewPage on the receive thread. Pages should still be
    // created before Start() so the very first ticks have something to render.
    protected ConcurrentDictionary<string, Screen> pages = new()
        {
              [DEFAULT_PAGE] = new Screen()
        };

    /// <summary>
    /// CDU scratchpad for Perf input pages (e.g. the A-10C takeoff page). Call <see cref="CduScratchpad.HandleKey"/>
    /// in the key handler only while the relevant page is active; call
    /// <see cref="CduScratchpad.Clear"/> when navigating away.
    /// </summary>
    protected CduScratchpad Scratchpad { get; } = new();

    /// <summary>
    /// Returns a boxed placeholder of <paramref name="width"/> '.' characters
    /// when <paramref name="value"/> is null, or the value right-padded to
    /// <paramref name="width"/> when set.
    /// </summary>
    protected static string BoxField(string? value, int width)
    {
        if (value is null) return new string('.', width);
        return value.Length >= width ? value[..width] : value.PadRight(width);
    }

    public AircraftListener(AircraftDescriptor descriptor, UserOptions options)
    {
        this.descriptor = descriptor;
        this.options = options;
        DCSBIOSControlLocator.DCSAircraft = DCSAircraft.GetAircraft(descriptor.EffectiveDcsBiosModuleId);
        _UpdateCounterDCSBIOSOutput = DCSBIOSOutput.GetUpdateCounter();

        _DisplayCDUTimer = new(_TICK_DISPLAY);
        _DisplayCDUTimer.Elapsed += (_, _) =>
        {
            // A render tick must never crash the app. When a device is unplugged its
            // USB stream dies (and is disposed on hot-removal), so Render can throw on
            // this timer thread until the listener is stopped — swallow and log it.
            try
            {
                var c = cdu;
                if (c != null)
                {
                    c.Render(pages[_currentPage]);
                }
            }
            catch (Exception ex)
            {
                App.Logger.Error(ex, "CDU render tick failed");
            }
        };

        _LedCDUTimer = new(_TICK_LEDS);
        _LedCDUTimer.Elapsed += (_, _) =>
        {
            // Same reasoning as the display tick: an unplugged device must not crash the app.
            try
            {
                cdu?.RenderLeds();
            }
            catch (Exception ex)
            {
                App.Logger.Error(ex, "CDU LED tick failed");
            }
        };
    }

    /// <summary>
    /// The grid this aircraft's display needs, or null to leave the panel on its own.
    /// The C-130J's CNI is 25 characters across where every other display here is 24,
    /// and the panels are addressable enough to run either.
    /// </summary>
    protected virtual (int Lines, int Columns)? ScreenSize => null;

    /// <summary>
    /// The green this aircraft's display uses, as RRGGBB, or null for the library's own.
    /// A module names its phosphor colour and they do not all agree, so an aircraft that
    /// knows its own says so here.
    /// </summary>
    protected virtual string? DisplayGreenRgb => null;

    internal void AttachCduContext(AircraftCduContext cduContext)
    {
        cdu = cduContext ?? throw new ArgumentNullException(nameof(cduContext));

        // A panel outlives the listener driving it, so an aircraft that wants its own
        // grid has to put the panel back on the default one for whatever comes next.
        var wanted = ScreenSize ?? cdu.Device.DefaultScreenSize;
        cdu.Device.SetScreenSize(wanted.Lines, wanted.Columns);

        // Same reasoning as the grid: the panel outlives the listener, so an aircraft that
        // does not name a colour puts the default back rather than inheriting the last one's.
        cdu.Device.Palette.Green.CopyFrom(
            PaletteColour.Parse(DisplayGreenRgb ?? Palette.DefaultGreenRgb));
        cdu.Device.RefreshPalette();

        // The default page is built by a field initialiser, before the panel is known,
        // so it carries the default grid. Rebuild it if this panel runs another one.
        var deviceScreen = cdu.Device.Screen;
        if (pages.TryGetValue(DEFAULT_PAGE, out var existing)
            && (existing.ColumnCount != deviceScreen.ColumnCount
                || existing.LineCount != deviceScreen.LineCount))
        {
            pages[DEFAULT_PAGE] = NewPage();
        }
    }

    /// <summary>
    /// Swaps the height in a font's filename for the one the panel can show. Descriptors
    /// name a font by aircraft, but the ceiling belongs to the panel - the MCDU's aperture
    /// is shorter than the PFP family's - and the same aircraft can appear on either.
    /// Font files are named "...-21x31.json", so the height is whatever follows the last
    /// 'x'. Falls back to the descriptor's own file when no variant exists.
    /// </summary>
    /// <param name="descriptorPath"></param>
    /// <param name="maxGlyphHeight"></param>
    private static string ResolveFontForPanel(string descriptorPath, int maxGlyphHeight)
    {
        var baseDir = AppContext.BaseDirectory;
        var fallback = Path.Combine(baseDir, descriptorPath);

        var marker = descriptorPath.LastIndexOf('x');
        if (marker < 0) return fallback;

        var prefix = descriptorPath[..(marker + 1)];
        for (var h = maxGlyphHeight; h > 0; h--)
        {
            var candidate = Path.Combine(baseDir, $"{prefix}{h}.json");
            if (File.Exists(candidate)) return candidate;
        }

        return fallback;
    }

    public void Start()
    {
        InitializeDcsBiosOutputs();

        // This aircraft's declared LED defaults, then whatever it computes for itself, then
        // the user's own bindings — each layer having the last word over the one before it.
        RegisterDefaultLedBindings();
        RegisterFrontpanelControls();
        RegisterUserLedBindings();

        if (cdu != null)
        {
            RegisterCduControls();
            // Load the correct font for this aircraft. Descriptor paths are relative to the
            // install folder (the .csproj copies the font JSONs next to the executable), so
            // anchor them to the base directory rather than the process working directory —
            // a shortcut with a different "Start in", or a launch from another process, would
            // otherwise leave the CDU showing whatever glyphs the device already held.
            var fontFile = ResolveFontForPanel(descriptor.FontFile, cdu.Device.MaxGlyphHeight);
            try
            {
                using var fileStream = new FileStream(fontFile, FileMode.Open, FileAccess.Read);
                using var reader = new StreamReader(fileStream);
                var fontJson = reader.ReadToEnd();
                var font = JsonConvert.DeserializeObject<McduFontFile>(fontJson);
                if (font != null)
                {
                    lock (cdu.State.SyncRoot)
                    {
                        cdu.State.Font = font;
                        cdu.State.FontDirty = true;
                    }
                }
                App.Logger.Info($"Loaded aircraft font: {fontFile}");
            }
            catch (Exception ex)
            {
                App.Logger.Error(ex, $"Failed to load font file: {fontFile}");
            }

            InitMcduBrightness();
        }

        BIOSEventHandler.AttachStringListener(this);
        BIOSEventHandler.AttachDataListener(this);
        BIOSEventHandler.AttachConnectionListener(this);

        cdu?.Render(pages[_currentPage]);
        _DisplayCDUTimer.Start();
        _LedCDUTimer.Start();

    }

    protected virtual void InitMcduBrightness()
    {
        if (options.DisableLightingManagement || cdu == null) return;
        SetCduBacklightBrightnessPercent(50);
        SetCduLedBrightnessPercent(100);
        SetCduDisplayBrightnessPercent(100);
    }

    public void Stop()
    {
        var capturedCdu = cdu;
        cdu = null;

        _DisplayCDUTimer.Stop();
        _LedCDUTimer.Stop();

        BIOSEventHandler.DetachConnectionListener(this);
        BIOSEventHandler.DetachDataListener(this);
        BIOSEventHandler.DetachStringListener(this);

        capturedCdu?.Cleanup();
    }

    protected bool HasCdu => cdu != null;
    protected ICdu? CduDevice => cdu?.Device;

    protected int GetDisplayBrightnessPercent()
    {
        if (cdu == null) return 100;
        lock (cdu.State.SyncRoot)
        {
            return cdu.State.DisplayBrightnessPercent;
        }
    }

    protected void SetCduDisplayBrightnessPercent(int value)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            cdu.State.DisplayBrightnessPercent = Math.Clamp(value, 0, 100);
            cdu.State.BrightnessDirty = true;
        }
    }

    protected void SetCduBacklightBrightnessPercent(int value)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            cdu.State.BacklightBrightnessPercent = Math.Clamp(value, 0, 100);
            cdu.State.BrightnessDirty = true;
        }
    }

    protected void SetCduLedBrightnessPercent(int value)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            cdu.State.LedBrightnessPercent = Math.Clamp(value, 0, 100);
            cdu.State.BrightnessDirty = true;
        }
    }

    /// <summary>
    /// Sets the MCDU annunciators this aircraft drives. LEDs the user has bound to a
    /// DCS-BIOS control of their own are left alone: a user binding wins over the
    /// aircraft's built-in one, the same rule the frontpanel renderers follow.
    /// </summary>
    protected void SetCduLeds(
        bool? fail = null,
        bool? fm1 = null,
        bool? fm2 = null,
        bool? fm = null,
        bool? ind = null,
        bool? rdy = null)
    {
        if (fail.HasValue) SetDefaultCduLed(McduLed.Fail, fail.Value);
        if (fm1.HasValue) SetDefaultCduLed(McduLed.Fm1, fm1.Value);
        if (fm2.HasValue) SetDefaultCduLed(McduLed.Fm2, fm2.Value);
        if (fm.HasValue) SetDefaultCduLed(McduLed.Fm, fm.Value);
        if (ind.HasValue) SetDefaultCduLed(McduLed.Ind, ind.Value);
        if (rdy.HasValue) SetDefaultCduLed(McduLed.Rdy, rdy.Value);
    }

    /// <summary>Writes an annunciator the aircraft owns, unless the user took it over.</summary>
    private void SetDefaultCduLed(McduLed led, bool on)
    {
        if (IsUserBound(led)) return;
        WriteCduLed(led, on);
    }

    /// <summary>
    /// Writes one annunciator. Only marks the LEDs dirty when a value actually changed: some
    /// aircraft read their lamps out of a shared register that DCS-BIOS resends whenever any
    /// bit in it moves, and a dirty flag costs a USB write on the next render tick.
    /// </summary>
    private void WriteCduLed(McduLed led, bool on)
    {
        var c = cdu;
        if (c == null) return;

        lock (c.State.SyncRoot)
        {
            var state = c.State;
            bool changed;

            switch (led)
            {
                case McduLed.Fail: changed = state.LedFail != on; state.LedFail = on; break;
                case McduLed.Fm1: changed = state.LedFm1 != on; state.LedFm1 = on; break;
                case McduLed.Fm2: changed = state.LedFm2 != on; state.LedFm2 = on; break;
                case McduLed.Fm: changed = state.LedFm != on; state.LedFm = on; break;
                case McduLed.Ind: changed = state.LedInd != on; state.LedInd = on; break;
                case McduLed.Rdy: changed = state.LedRdy != on; state.LedRdy = on; break;
                default: changed = false; break;
            }

            if (changed) state.LedsDirty = true;
        }
    }

    // ── User LED bindings (ledmappings.json) ─────────────────────────────────

    /// <summary>MCDU annunciators the user bound on this aircraft.</summary>
    private readonly HashSet<McduLed> _userBoundCduLeds = new();

    private bool IsUserBound(McduLed led) => _userBoundCduLeds.Contains(led);

    /// <summary>
    /// Registers this aircraft's declared LED defaults (see <see cref="LedDefaults"/>). Keeping
    /// them in a table rather than in each listener means the LED editor can show what a LED
    /// already follows without starting the aircraft, and cannot fall out of step with what the
    /// bridge actually does.
    /// </summary>
    private void RegisterDefaultLedBindings()
    {
        var defaults = LedDefaults.For(descriptor);

        foreach (var signal in defaults.Signals)
        {
            // Each control keeps its own last value, so a signal read off several of them
            // reflects all of them and not just whichever moved last.
            var lit = new bool[signal.Controls.Count];

            for (var i = 0; i < signal.Controls.Count; i++)
            {
                var slot = i;
                TryRegisterDefault(signal.Controls[slot], v =>
                {
                    lit[slot] = v != 0;
                    FlightDeck.SetSignal(signal.Signal, Array.IndexOf(lit, true) >= 0);
                });
            }
        }

        foreach (var led in defaults.McduLeds)
            TryRegisterDefault(led.Control, v => SetDefaultCduLed(led.Led, v != 0));
    }

    /// <summary>
    /// Registers a declared default. Same reasoning as the user bindings: a control the
    /// installed DCS-BIOS no longer defines must cost that one LED, not the bridge.
    /// </summary>
    private void TryRegisterDefault(string controlId, Action<uint> handler)
    {
        try
        {
            RegisterUInt(controlId, handler);
        }
        catch (Exception ex)
        {
            App.Logger.Warn(ex, $"Built-in LED default ignored: '{controlId}' is not a control of " +
                                $"the {descriptor.DisplayName}");
        }
    }

    /// <summary>
    /// Registers the user's own LED bindings for this aircraft. Frontpanel LEDs land in
    /// <see cref="FlightDeck"/> for the renderers to pick up; MCDU LEDs are written straight
    /// to the CDU state. Called from <see cref="Start"/> for every aircraft — no listener
    /// has to know the feature exists.
    /// </summary>
    private void RegisterUserLedBindings()
    {
        var bindings = LedMappingStore.ForAircraft(descriptor.DisplayName);
        if (bindings.Count == 0) return;

        // User LEDs are lighting like any other: SimApp Pro users own the panels.
        if (options.DisableLightingManagement)
        {
            App.Logger.Info($"{bindings.Count} user LED binding(s) skipped: lighting management is disabled");
            return;
        }

        foreach (var binding in bindings)
        {
            if (binding.Device == LedDeviceFamily.Mcdu)
            {
                if (LedCatalog.ParseMcduLed(binding.Led) is not McduLed led) continue;

                // Only claim the LED once the control resolved: a binding that failed to
                // register must not suppress the aircraft's own use of that annunciator.
                if (TryRegisterUserLed(binding, v => WriteCduLed(led, binding.IsOn(v))))
                    _userBoundCduLeds.Add(led);
            }
            else
            {
                TryRegisterUserLed(binding, v => FlightDeck.SetUserLed(binding.Device, binding.Led, binding.IsOn(v)));
            }
        }
    }

    /// <summary>
    /// Registers a handler for a user-supplied control id. Unlike the hard-coded ids the
    /// listeners use, this one comes from a file the user (or whoever shared it) wrote, and
    /// <see cref="DCSBIOSControlLocator.GetUIntDCSBIOSOutput"/> throws on an identifier the
    /// module does not define — a stale binding after a DCS-BIOS update would otherwise take
    /// the whole bridge down. Report it and carry on with the rest.
    /// </summary>
    private bool TryRegisterUserLed(LedBinding binding, Action<uint> handler)
    {
        try
        {
            RegisterUInt(binding.Control, handler);
            App.Logger.Info($"User LED binding: {binding.Device}/{binding.Led} follows {binding.Control}");
            return true;
        }
        catch (Exception ex)
        {
            App.Logger.Warn(ex, $"User LED binding ignored: {binding.Device}/{binding.Led} cannot follow " +
                                $"'{binding.Control}' on the {descriptor.DisplayName}");
            return false;
        }
    }



    protected virtual void InitializeDcsBiosOutputs() { }
    protected abstract void RegisterCduControls();

    /// <summary>
    /// Registers DCS-BIOS handlers that populate <see cref="FlightDeck"/>.
    /// Implementations write semantic values only — no frontpanel device types.
    /// Called unconditionally: whether any frontpanel is connected is the
    /// renderers' concern, not the aircraft's.
    /// </summary>
    protected abstract void RegisterFrontpanelControls();

    public void DcsBiosConnectionActive(object sender, DCSBIOSConnectionEventArgs e)
    {
    }

    /// <summary>
    /// Pages are built at the panel's own grid size, not the default one. A listener
    /// driving a wider CDU would otherwise compose into a 24 column buffer and lose
    /// its last column on the way to the device.
    /// </summary>
    private Screen NewPage() =>
        cdu == null
            ? new Screen()
            : new Screen(cdu.Device.Screen.LineCount, cdu.Device.Screen.ColumnCount);

    protected Compositor GetCompositor(string pageName)
    {
        return new Compositor(pages.GetOrAdd(pageName, _ => NewPage()));
    }

    protected Screen AddNewPage(string pageName)
    {
        return pages.GetOrAdd(pageName, _ => NewPage());
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
        if (_disposed) return;

        if (disposing)
        {
            Stop();
            _DisplayCDUTimer.Dispose();
            _LedCDUTimer.Dispose();
        }

        _disposed = true;
    }

    protected void UpdateCounter(uint address, uint data)
    {
        lock (_UpdateCounterLockObject)
        {
            if (_UpdateCounterDCSBIOSOutput != null && _UpdateCounterDCSBIOSOutput.Address == address)
            {
                var newCount = _UpdateCounterDCSBIOSOutput.GetUIntValue(data);
                var previousCount = _Count;

                if (!_HasSyncOnce)
                {
                    _HasSyncOnce = true;
                }
                else if (!((newCount == 0 && previousCount == 255) || newCount - previousCount == 1))
                {
                    App.Logger.Warn($"UpdateCounter: Address {address} has unexpected value {data}. Expected {previousCount + 1}.");
                }

                _Count = newCount;
            }
        }
    }
}
