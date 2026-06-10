using ClassLibraryCommon;
using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using Newtonsoft.Json;
using System.IO;
using Timer = System.Timers.Timer;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge.Aircrafts;

internal abstract class AircraftListener : IDcsBiosListener, IDisposable
{
    private static readonly double _TICK_DISPLAY = 100;
    private readonly Timer _DisplayCDUTimer;
    protected ICdu? mcdu;
    protected FrontpanelHub frontpanelHub;
    protected IFrontpanelState? frontpanelState;
    protected IFrontpanelLeds? frontpanelLeds;

    private bool _disposed;

    private readonly DCSBIOSOutput _UpdateCounterDCSBIOSOutput;
    private static readonly object _UpdateCounterLockObject = new();
    private bool _HasSyncOnce;
    private uint _Count;

    protected readonly UserOptions options;

    protected const string DEFAULT_PAGE = "default";
    protected string _currentPage = DEFAULT_PAGE;

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

    public AircraftListener(ICdu? mcdu, int aircraftNumber, UserOptions options, FrontpanelHub frontpanelHub)
    {
        this.mcdu = mcdu;
        this.frontpanelHub = frontpanelHub ?? throw new ArgumentNullException(nameof(frontpanelHub));
        this.options = options;
        DCSBIOSControlLocator.DCSAircraft = DCSAircraft.GetAircraft(aircraftNumber);
        _UpdateCounterDCSBIOSOutput = DCSBIOSOutput.GetUpdateCounter();

        _DisplayCDUTimer = new(_TICK_DISPLAY);
        _DisplayCDUTimer.Elapsed += (_, _) =>
        {
            if (this.mcdu != null)
            {
                this.mcdu.Screen.CopyFrom(pages[_currentPage]);
                this.mcdu.RefreshDisplay();
            }

            if (frontpanelHub.HasFrontpanels && frontpanelState != null)
            {
                frontpanelHub.UpdateDisplay(frontpanelState);
            }

            if (frontpanelHub.HasFrontpanels && frontpanelLeds != null)
            {
                frontpanelHub.UpdateLeds(frontpanelLeds);
            }
        };

        // Initialize frontpanel state and LEDs based on connected adapters
        // PAP3 and PDC-3N are designed to work together (Boeing 737 style)
        // FCU and EFIS work together (Airbus style, but as single device with extension)
        // All frontpanels receive the same brightness updates
        if (frontpanelHub.HasFrontpanels)
        {
            var adapters = frontpanelHub.Adapters.ToList();
            
            // Check for FCU/EFIS (single device or device with extension)
            var hasFcuEfis = adapters.Any(a => a is FcuEfisAdapter);
            
            // Check for PAP3 (primary autopilot panel)
            var hasPap3 = adapters.Any(a => a is Pap3Adapter);
            
            // Check for PDC-3N (panel display controller, used with PAP3)
            var hasPdc3 = adapters.Any(a => a is Pdc3Adapter);

            // Check for AGP32 clock/gear panel
            var hasAgp32 = adapters.Any(a => a is Agp32Adapter);

            if (hasFcuEfis)
            {
                frontpanelState = new FcuEfisState();
                frontpanelLeds = new FcuEfisLeds();
                InitializeFrontpanelBrightness(128, 255, 255);
                App.Logger.Info("FCU/EFIS device detected and initialized");
            }
            else if (hasPap3)
            {
                frontpanelState = new Pap3State();
                frontpanelLeds = new Pap3Leds();
                
                // Initialize PAP3 state with default values so displays show something
                // Empty/null values cause displays to turn off
                if (frontpanelState is Pap3State pap3State)
                {
                    pap3State.Speed = 0;
                    pap3State.Heading = 0;
                    pap3State.Altitude = 0;
                    pap3State.VerticalSpeed = 0;
                }
                
                // PAP3 has separate brightness for: panel backlight, LCD displays, LED markers
                // Set LCD displays (middle parameter) to full brightness for visibility
                InitializeFrontpanelBrightness(255, 255, 255);
                
                if (hasPdc3)
                {
                    App.Logger.Info("PAP3 + PDC-3N devices detected and initialized (Boeing 737 configuration)");
                }
                else
                {
                    App.Logger.Info("PAP3 device detected and initialized");
                }
            }
            else if (hasAgp32)
            {
                frontpanelState = new Agp32State();
                frontpanelLeds = new Agp32State.Agp32Leds();
                InitializeFrontpanelBrightness(255, 255, 255);
                App.Logger.Info("AGP32 device detected and initialized");
            }
            else if (hasPdc3)
            {
                // PDC-3N alone: only brightness control, no display or LEDs
                InitializeFrontpanelBrightness(255, 255, 255);
                App.Logger.Info("PDC-3N device detected and initialized (brightness control only)");
            }
            else
            {
                var firstAdapter = adapters.FirstOrDefault();
                if (firstAdapter == null)
                {
                    App.Logger.Warn("FrontpanelHub reports frontpanels, but no adapters were found.");
                }
                else
                {
                    App.Logger.Warn($"Unknown frontpanel adapter type: {firstAdapter.GetType().Name}");
                }
            }
        }
    }

    public void Start()
    {
        InitializeDcsBiosOutputs();

        if (frontpanelState != null) RegisterFrontpanelControls();

        if (mcdu != null)
        {
            RegisterMcduControls();
            // Load the correct font for this aircraft
            var fontFile = GetFontFile();
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

    private void InitializeFrontpanelBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight)
    {
        if (options.DisableLightingManagement || !frontpanelHub.HasFrontpanels) return;
        
        App.Logger.Info($"Setting frontpanel brightness: Panel={panelBacklight}, LCD={lcdBacklight}, LED={ledBacklight}");
        frontpanelHub.SetBrightness(panelBacklight, lcdBacklight, ledBacklight);
        
        // For PAP3, send an initial display update to "wake up" the displays
        // Some devices require at least one display update before brightness takes effect
        if (frontpanelState != null)
        {
            App.Logger.Info("Sending initial frontpanel display update");
            frontpanelHub.UpdateDisplay(frontpanelState);
        }
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

    protected abstract string GetFontFile();
    protected abstract string GetAircraftName();
    protected abstract void InitializeDcsBiosOutputs();
    protected abstract void RegisterMcduControls();
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
                    Console.WriteLine($"UpdateCounter: Address {address} has unexpected value {data}. Expected {_Count + 1}.");
                }
            }
        }
    }
}
