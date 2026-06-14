using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal enum DisplayMode { DED, NAV, RWR }

internal class F16C_Listener : AircraftListener
{
    private const string NAV_PAGE = "NAV";
    private const string RWR_PAGE = "RWR";

    private DCSBIOSOutput? _PRI_CONSOLES_BRT;
    private DCSBIOSOutput? _LIGHT_MASTER_CAUTION;
    private DCSBIOSOutput? _PRI_DATA_DISPLAY_BRT;

    private DCSBIOSOutput? _LIGHT_GEAR_L;
    private DCSBIOSOutput? _LIGHT_GEAR_N;
    private DCSBIOSOutput? _LIGHT_GEAR_R;
    private DCSBIOSOutput? _LIGHT_GEAR_WARN;

    private readonly Key _dedDisplayKey;
    private readonly Key _navDisplayKey;
    private readonly Key _rwrDisplayKey;

    private readonly F16C_Ded_Page _dedPage = new();
    private readonly F16C_Nav_Page _navPage = new();
    private readonly F16C_Rwr_Page _rwrPage = new();

    public F16C_Listener(UserOptions options)
        : base(AircraftRegistry.F16C, options)
    {
        _dedDisplayKey = Enum.TryParse<Key>(options.F16CPrevDisplayKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;
        _navDisplayKey = Enum.TryParse<Key>(options.F16CNextDisplayKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _rwrDisplayKey = Enum.TryParse<Key>(options.F16CRwrDisplayKey, out var rwrKey)
            ? rwrKey
            : Key.LineSelectRight1;

        AddNewPage(NAV_PAGE);
        AddNewPage(RWR_PAGE);
    }

    ~F16C_Listener() => Dispose(false);

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == _dedDisplayKey)
        {
            SwitchDisplay(DisplayMode.DED);
        }
        else if (e.Key == _navDisplayKey)
        {
            SwitchDisplay(DisplayMode.NAV);
        }
        else if (e.Key == _rwrDisplayKey)
        {
            SwitchDisplay(DisplayMode.RWR);
        }
    }

    public void SwitchDisplay(DisplayMode newMode)
    {
        switch (newMode)
        {
            case DisplayMode.DED:
                _currentPage = DEFAULT_PAGE;
                _dedPage.InvalidateCache();
                _dedPage.Render(GetCompositor(DEFAULT_PAGE));
                break;
            case DisplayMode.NAV:
                _currentPage = NAV_PAGE;
                _navPage.Render(GetCompositor(NAV_PAGE));
                break;
            case DisplayMode.RWR:
                _currentPage = RWR_PAGE;
                _rwrPage.Render(GetCompositor(RWR_PAGE));
                break;
        }
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        if (!options.DisableLightingManagement)
        {
            Register(_PRI_CONSOLES_BRT, v =>
                SetBacklightBrightnessPercent((int)(v * 100 / _PRI_CONSOLES_BRT!.MaxValue)));
            Register(_PRI_DATA_DISPLAY_BRT, v =>
                SetDisplayBrightnessPercent((int)(v * 100 / _PRI_DATA_DISPLAY_BRT!.MaxValue)));
        }

        Register(_LIGHT_MASTER_CAUTION, v => SetCduLeds(fail: v == 1));

        _dedPage.RegisterControls(Register, RegisterString, () => GetCompositor(DEFAULT_PAGE));
        _navPage.RegisterControls(Register, RegisterString, () => GetCompositor(NAV_PAGE));
        _rwrPage.RegisterControls(Register, RegisterString, () => GetCompositor(RWR_PAGE));
    }

    protected override void RegisterFrontpanelControls() {

        Register(_LIGHT_GEAR_L, v => FlightDeck.GearLeftDown = v == 1);
        Register(_LIGHT_GEAR_N, v => FlightDeck.GearNoseDown = v == 1);
        Register(_LIGHT_GEAR_R, v => FlightDeck.GearRightDown = v == 1);

        Register(_LIGHT_GEAR_WARN, v => FlightDeck.GearWarning = v == 1);
    }

    protected override void InitializeDcsBiosOutputs()
    {
        _PRI_CONSOLES_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_CONSOLES_BRT_KNB");
        _LIGHT_MASTER_CAUTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_MASTER_CAUTION");
        _PRI_DATA_DISPLAY_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_DATA_DISPLAY_BRT_KNB");

        _LIGHT_GEAR_L = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_GEAR_L");
        _LIGHT_GEAR_N = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_GEAR_N");
        _LIGHT_GEAR_R = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_GEAR_R");

        _LIGHT_GEAR_WARN = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_GEAR_WARN");
            
        _dedPage.InitializeControls();
        _navPage.InitializeControls();
        _rwrPage.InitializeControls();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
        }

        base.Dispose(disposing);
    }
}
