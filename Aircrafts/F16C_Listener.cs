using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal enum DisplayMode { DED, NAV, RWR }

internal class F16C_Listener : AircraftListener
{
    private volatile DisplayMode _currentDisplay = DisplayMode.DED;

    private DCSBIOSOutput? _PRI_CONSOLES_BRT;
    private DCSBIOSOutput? _LIGHT_MASTER_CAUTION;
    private DCSBIOSOutput? _PRI_DATA_DISPLAY_BRT;

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
        if (newMode == DisplayMode.DED)
        {
            _dedPage.InvalidateCache();
        }

        _currentDisplay = newMode;
        RefreshActiveDisplay();
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }
    }

    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        _PRI_CONSOLES_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_CONSOLES_BRT_KNB");
        _LIGHT_MASTER_CAUTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_MASTER_CAUTION");
        _PRI_DATA_DISPLAY_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_DATA_DISPLAY_BRT_KNB");

        _dedPage.InitializeControls();
        _navPage.InitializeControls();
        _rwrPage.InitializeControls();
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);

            if (HasCdu && !options.DisableLightingManagement)
            {
                if (_PRI_CONSOLES_BRT != null && e.Address == _PRI_CONSOLES_BRT.Address)
                {
                    SetBacklightBrightnessPercent(
                        (int)(_PRI_CONSOLES_BRT.GetUIntValue(e.Data) * 100 / _PRI_CONSOLES_BRT.MaxValue));
                }

                if (_PRI_DATA_DISPLAY_BRT != null && e.Address == _PRI_DATA_DISPLAY_BRT.Address)
                {
                    SetDisplayBrightnessPercent(
                        (int)(_PRI_DATA_DISPLAY_BRT.GetUIntValue(e.Data) * 100 / _PRI_DATA_DISPLAY_BRT.MaxValue));
                }
            }

            if (!options.DisableLightingManagement && _PRI_CONSOLES_BRT != null && e.Address == _PRI_CONSOLES_BRT.Address)
            {
                FlightDeck.ConsoleBrightness =
                    (byte)(_PRI_CONSOLES_BRT.GetUIntValue(e.Data) * 255 / _PRI_CONSOLES_BRT.MaxValue);
            }

            if (HasCdu && _LIGHT_MASTER_CAUTION != null && e.Address == _LIGHT_MASTER_CAUTION.Address)
            {
                SetCduLeds(fail: _LIGHT_MASTER_CAUTION.GetUIntValue(e.Data) == 1);
            }

            bool navChanged = _navPage.ProcessData(e);
            bool rwrChanged = _rwrPage.ProcessData(e);

            if ((_currentDisplay == DisplayMode.NAV && navChanged) ||
                (_currentDisplay == DisplayMode.RWR && rwrChanged))
            {
                RefreshActiveDisplay();
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "F-16C: Failed to process DCS-BIOS data");
        }
    }

    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        try
        {
            switch (_currentDisplay)
            {
                case DisplayMode.DED:
                    if (_dedPage.ProcessData(e))
                    {
                        _dedPage.Render(GetCompositor(DEFAULT_PAGE));
                    }
                    break;

                case DisplayMode.NAV:
                    if (_navPage.ProcessData(e))
                    {
                        _navPage.Render(GetCompositor(DEFAULT_PAGE));
                    }
                    break;

                case DisplayMode.RWR:
                    if (_rwrPage.ProcessData(e))
                    {
                        _rwrPage.Render(GetCompositor(DEFAULT_PAGE));
                    }
                    break;
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "F-16C: Failed to process DCS-BIOS string data");
        }
    }

    private void RefreshActiveDisplay()
    {
        var output = GetCompositor(DEFAULT_PAGE);

        switch (_currentDisplay)
        {
            case DisplayMode.DED:
                _dedPage.Render(output);
                break;
            case DisplayMode.NAV:
                _navPage.Render(output);
                break;
            case DisplayMode.RWR:
                _rwrPage.Render(output);
                break;
        }
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
