using ClassLibraryCommon;
using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using Newtonsoft.Json;
using System.IO;
using Timer = System.Timers.Timer;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal abstract class AircraftListener : IDcsBiosListener, IDisposable
{
    private static readonly double _TICK_DISPLAY = 100;
    private readonly Timer _DisplayCDUTimer;
    protected ICdu? mcdu;

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

    public AircraftListener(ICdu? mcdu, AircraftDescriptor descriptor, UserOptions options)
    {
        this.mcdu = mcdu;
        this.descriptor = descriptor;
        this.options = options;
        DCSBIOSControlLocator.DCSAircraft = DCSAircraft.GetAircraft(descriptor.ModuleId);
        _UpdateCounterDCSBIOSOutput = DCSBIOSOutput.GetUpdateCounter();

        _DisplayCDUTimer = new(_TICK_DISPLAY);
        _DisplayCDUTimer.Elapsed += (_, _) =>
        {
            if (this.mcdu != null)
            {
                this.mcdu.Screen.CopyFrom(pages[_currentPage]);
                this.mcdu.RefreshDisplay();
            }
        };
    }

    public void Start()
    {
        InitializeDcsBiosOutputs();

        RegisterFrontpanelControls();

        if (mcdu != null)
        {
            RegisterCduControls();
            // Load the correct font for this aircraft
            var fontFile = descriptor.FontFile;
            try
            {
                using var fileStream = new FileStream(fontFile, FileMode.Open, FileAccess.Read);
                using var reader = new StreamReader(fileStream);
                var fontJson = reader.ReadToEnd();
                mcdu.UseFont(JsonConvert.DeserializeObject<McduFontFile>(fontJson), true);
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
        if (disabledBrightness || mcdu == null) return;
        mcdu.BacklightBrightnessPercent = 100;
        mcdu.LedBrightnessPercent = 100;
        mcdu.DisplayBrightnessPercent = 100;
    }

    public void Stop()
    {
        _DisplayCDUTimer.Stop();

        BIOSEventHandler.DetachConnectionListener(this);
        BIOSEventHandler.DetachDataListener(this);
        BIOSEventHandler.DetachStringListener(this);

        if (mcdu != null)
        {
            mcdu.Output.Clear();
            mcdu.Cleanup();
            mcdu.RefreshDisplay();
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
                if (!_HasSyncOnce)
                {
                    _Count = newCount;
                    _HasSyncOnce = true;
                    return;
                }

                if (newCount == 0 && _Count == 255 || newCount - _Count == 1)
                {
                    _Count = newCount;
                }
                else if (newCount - _Count != 1)
                {
                    _Count = newCount;
                    App.Logger.Warn($"UpdateCounter: Address {address} has unexpected value {data}. Expected {_Count + 1}.");
                }
            }
        }
    }
}
