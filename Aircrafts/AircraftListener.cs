using ClassLibraryCommon;
using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using Newtonsoft.Json;
using System.IO;
using Timer = System.Timers.Timer;
using WwDevicesDotNet;
using WWCduDcsBiosBridge.Devices.Cdu;

namespace WWCduDcsBiosBridge.Aircrafts;

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

    // Enregistre un handler pour un output entier
    protected void Register(DCSBIOSOutput? output, Action<uint> handler)
    {
        if (output is null) return;
        if (!_dataHandlers.TryGetValue(output.Address, out var list))
            _dataHandlers[output.Address] = list = new();
        // capture l'output pour décoder la valeur correctement
        list.Add(data => handler(output.GetUIntValue(data)));
    }

    // Enregistre un handler pour un output string
    protected void RegisterString(DCSBIOSOutput? output, Action<string> handler)
    {
        if (output is null) return;
        if (!_stringHandlers.TryGetValue(output.Address, out var list))
            _stringHandlers[output.Address] = list = new();
        list.Add(handler);
    }

    // Dispatch unique, plus de if-chains dans les classes filles.
    // virtual : les aircrafts non encore migrés vers Register continuent à override.
    public virtual void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
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

    public virtual void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
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


    protected Dictionary<string, Screen> pages = new()
        {
              {DEFAULT_PAGE, new Screen() }
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

            InitMcduBrightness(options.DisableLightingManagement);
        }

        BIOSEventHandler.AttachStringListener(this);
        BIOSEventHandler.AttachDataListener(this);
        BIOSEventHandler.AttachConnectionListener(this);

        _DisplayCDUTimer.Start();

    }

    private void InitMcduBrightness(bool disabledBrightness)
    {
        if (disabledBrightness || cdu == null) return;
        SetBacklightBrightnessPercent(100);
        SetLedBrightnessPercent(100);
        SetDisplayBrightnessPercent(100);
    }

    public void Stop()
    {
        _DisplayCDUTimer.Stop();

        BIOSEventHandler.DetachConnectionListener(this);
        BIOSEventHandler.DetachDataListener(this);
        BIOSEventHandler.DetachStringListener(this);

        cdu?.Cleanup();
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

    protected void SetDisplayBrightnessPercent(int value)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            cdu.State.DisplayBrightnessPercent = Math.Clamp(value, 0, 100);
            cdu.State.BrightnessDirty = true;
        }
    }

    protected void SetBacklightBrightnessPercent(int value)
    {
        if (cdu == null) return;
        lock (cdu.State.SyncRoot)
        {
            cdu.State.BacklightBrightnessPercent = Math.Clamp(value, 0, 100);
            cdu.State.BrightnessDirty = true;
        }
    }

    protected void SetLedBrightnessPercent(int value)
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

    protected abstract void InitializeDcsBiosOutputs();
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
        if (!pages.ContainsKey(pageName))
        {
            pages[pageName] = new Screen();
        }
        return new Compositor(pages[pageName]);
    }

    protected Screen AddNewPage(string pageName)
    {
        if (!pages.ContainsKey(pageName))
        {
            pages[pageName] = new Screen();
        }

        return pages[pageName];
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
