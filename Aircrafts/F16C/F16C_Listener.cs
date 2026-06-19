using DCS_BIOS.EventArgs;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal enum DisplayMode { DED, NAV, RWR }

internal partial class F16C_Listener : AircraftListener
{
    private const string NAV_PAGE = "NAV";
    private const string RWR_PAGE = "RWR";

    private readonly Key _dedDisplayKey;
    private readonly Key _navDisplayKey;
    private readonly Key _rwrDisplayKey;

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
                InvalidateDedCache();
                RenderDed();
                break;
            case DisplayMode.NAV:
                _currentPage = NAV_PAGE;
                RenderNav();
                break;
            case DisplayMode.RWR:
                _currentPage = RWR_PAGE;
                RenderRwr();
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
            RegisterUInt("PRI_CONSOLES_BRT_KNB", (ctrl, v) =>
                SetBacklightBrightnessPercent((int)(v * 100 / ctrl.MaxValue)));
            RegisterUInt("PRI_DATA_DISPLAY_BRT_KNB", (ctrl, v) =>
                SetDisplayBrightnessPercent((int)(v * 100 / ctrl.MaxValue)));
        }

        RegisterUInt("LIGHT_MASTER_CAUTION", v => SetCduLeds(fail: v == 1));

        RegisterDedControls();
        RegisterNavControls();
        RegisterRwrControls();
    }

    protected override void RegisterFrontpanelControls()
    {
        RegisterUInt("LIGHT_GEAR_L",    v => FlightDeck.GearLeftDown = v == 1);
        RegisterUInt("LIGHT_GEAR_N",    v => FlightDeck.GearNoseDown = v == 1);
        RegisterUInt("LIGHT_GEAR_R",    v => FlightDeck.GearRightDown = v == 1);
        RegisterUInt("LIGHT_GEAR_WARN", v => FlightDeck.GearWarning = v == 1);
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
