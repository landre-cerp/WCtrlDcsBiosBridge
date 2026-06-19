using DCS_BIOS.EventArgs;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class FA18C_Listener : AircraftListener
{
    private const string IFEI_PAGE = "IFEI";

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

        RegisterUInt("COCKKPIT_LIGHT_MODE_SW", v => _lightMode = v);
        RegisterUInt("MASTER_CAUTION_LT", v =>
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

    protected override void RegisterFrontpanelControls()
    {
        RegisterUInt("FLP_LG_LEFT_GEAR_LT",    v => FlightDeck.GearLeftDown = v == 1);
        RegisterUInt("FLP_LG_RIGHT_GEAR_LT",   v => FlightDeck.GearRightDown = v == 1);
        RegisterUInt("FLP_LG_NOSE_GEAR_LT",    v => FlightDeck.GearNoseDown = v == 1);
        RegisterUInt("LANDING_GEAR_HANDLE_LT", v => FlightDeck.GearWarning = v == 1);
        RegisterUInt("FLP_LG_HALF_FLAPS_LT",   v => FlightDeck.LedAutoBrkLoDecel = v == 1);
        RegisterUInt("FLP_LG_FULL_FLAPS_LT",   v => FlightDeck.LedAutoBrkMedDecel = v == 1);

        _ifeiPage.RegisterFrontPanelControls(RegisterString, FlightDeck);
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
