using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WWCduDcsBiosBridge.Frontpanels;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class FA18C_Listener : AircraftListener
{
    private const string IFEI_PAGE = "IFEI";

    private DCSBIOSOutput? MASTER_CAUTION_LT;
    private DCSBIOSOutput? _cockpitLightModeSw;

    uint _masterCaution = 0;
    uint _lightMode = 0; // 2=NVG, 1=NITE, 0=DAY

    private readonly FA18C_UFC_Page _ufcPage = new();
    private readonly FA18C_IFEI_Page _ifeiPage = new();

    private readonly Key _nextPageKey;
    private readonly Key _prevPageKey;

    protected override string GetFontFile() => "resources/a10c-font-21x31.json";

    protected override string GetAircraftName() => SupportedAircrafts.FA18C_Name;

    public FA18C_Listener(ICdu? mcdu, UserOptions options)
        : base(mcdu, SupportedAircrafts.FA18C, options, FrontpanelHub.CreateEmpty())
    {
        _nextPageKey = Enum.TryParse<Key>(options.NextPageKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _prevPageKey = Enum.TryParse<Key>(options.PrevPageKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;

        AddNewPage(IFEI_PAGE);

        if (mcdu != null)
        {
            mcdu.KeyDown += HandleKeyDown;
        }
    }

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == _nextPageKey)
        {
            _currentPage = IFEI_PAGE;
            _ifeiPage.Render(GetCompositor(IFEI_PAGE), _lightMode);
        }
        else if (e.Key == _prevPageKey)
        {
            _currentPage = DEFAULT_PAGE;
        }
    }

    protected override void InitializeDcsBiosControls()
    {
        MASTER_CAUTION_LT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MASTER_CAUTION_LT");
        _cockpitLightModeSw = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("COCKKPIT_LIGHT_MODE_SW");

        _ufcPage.InitializeControls();
        _ifeiPage.InitializeControls();
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);

            if (_cockpitLightModeSw != null && e.Address.Equals(_cockpitLightModeSw.Address))
            {
                _lightMode = _cockpitLightModeSw.GetUIntValue(e.Data);
            }

            if (
                mcdu != null
                && MASTER_CAUTION_LT != null
                && e.Address.Equals(MASTER_CAUTION_LT.Address)
            )
            {
                uint newMasterCaution = MASTER_CAUTION_LT.GetUIntValue(e.Data);
                if (_masterCaution != newMasterCaution)
                {
                    _masterCaution = newMasterCaution;
                    mcdu.Leds.Fail = _masterCaution != 0;
                    mcdu.RefreshLeds();
                }
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS data");
        }
    }

    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        try
        {
            if (_currentPage == DEFAULT_PAGE)
            {
                _ufcPage.ProcessData(e);
                _ufcPage.Render(GetCompositor(DEFAULT_PAGE));
            }
            else if (_currentPage == IFEI_PAGE)
            {
                _ifeiPage.ProcessData(e);
                _ifeiPage.Render(GetCompositor(IFEI_PAGE), _lightMode);
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS string data");
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && mcdu != null)
        {
            mcdu.KeyDown -= HandleKeyDown;
        }
        base.Dispose(disposing);
    }
}
