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
using WCtrlDcsBiosBridge.Devices.Cdu;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal abstract class AircraftListener : IDcsBiosListener, IDisposable
{
    private static readonly double _TICK_DISPLAY = 100;
    private readonly Timer _DisplayCDUTimer;
    protected AircraftCduContext? cdu;

    private bool _disposed;

    private readonly DCSBIOSOutput _UpdateCounterDCSBIOSOutput;
    private static readonly object _UpdateCounterLockObject = new();
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

    public AircraftListener(AircraftDescriptor descriptor, UserOptions options)
    {
        this.descriptor = descriptor;
        this.options = options;
        DCSBIOSControlLocator.DCSAircraft = DCSAircraft.GetAircraft(descriptor.ModuleId);
        _UpdateCounterDCSBIOSOutput = DCSBIOSOutput.GetUpdateCounter();

        _DisplayCDUTimer = new(_TICK_DISPLAY);
        _DisplayCDUTimer.Elapsed += (_, _) =>
        {
            if (cdu != null)
            {
                cdu.Render(pages[_currentPage]);
            }
        };
    }

    internal void AttachCduContext(AircraftCduContext cduContext)
    {
        cdu = cduContext ?? throw new ArgumentNullException(nameof(cduContext));
    }

    public void Start()
    {
        InitializeDcsBiosOutputs();

        RegisterFrontpanelControls();

        if (cdu != null)
        {
            RegisterCduControls();
            // Load the correct font for this aircraft
            var fontFile = descriptor.FontFile;
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
        cdu = null;   // prevent any thread-pool timer tick queued before Stop() from rendering

        _DisplayCDUTimer.Stop();

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

    protected void SetCduLeds(
        bool? fail = null,
        bool? fm1 = null,
        bool? fm2 = null,
        bool? fm = null,
        bool? ind = null,
        bool? rdy = null)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            if (fail.HasValue) cdu.State.LedFail = fail.Value;
            if (fm1.HasValue) cdu.State.LedFm1 = fm1.Value;
            if (fm2.HasValue) cdu.State.LedFm2 = fm2.Value;
            if (fm.HasValue) cdu.State.LedFm = fm.Value;
            if (ind.HasValue) cdu.State.LedInd = ind.Value;
            if (rdy.HasValue) cdu.State.LedRdy = rdy.Value;
            cdu.State.LedsDirty = true;
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

    protected Compositor GetCompositor(string pageName)
    {
        return new Compositor(pages.GetOrAdd(pageName, _ => new Screen()));
    }

    protected Screen AddNewPage(string pageName)
    {
        return pages.GetOrAdd(pageName, _ => new Screen());
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
