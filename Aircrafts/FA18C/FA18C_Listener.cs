using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
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

    public FA18C_Listener(UserOptions options)
        : base(AircraftRegistry.FA18C, options)
    {
        _nextPageKey = Enum.TryParse<Key>(options.NextPageKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _prevPageKey = Enum.TryParse<Key>(options.PrevPageKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;

        AddNewPage(IFEI_PAGE);
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
            _ufcPage.Render(GetCompositor(DEFAULT_PAGE));
        }
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        Register(_cockpitLightModeSw, v => _lightMode = v);
        Register(MASTER_CAUTION_LT, v =>
        {
            if (_masterCaution != v)
            {
                _masterCaution = v;
                SetCduLeds(fail: _masterCaution != 0);
            }
        });

        _ufcPage.RegisterControls(RegisterString, () => _ufcPage.Render(GetCompositor(DEFAULT_PAGE)));
        _ifeiPage.RegisterControls(RegisterString, () => _ifeiPage.Render(GetCompositor(IFEI_PAGE), _lightMode));
    }
    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        MASTER_CAUTION_LT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MASTER_CAUTION_LT");
        _cockpitLightModeSw = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("COCKKPIT_LIGHT_MODE_SW");

        _ufcPage.InitializeControls();
        _ifeiPage.InitializeControls();
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
