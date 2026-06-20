using DCS_BIOS.EventArgs;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class FA18C_Listener : AircraftListener
{
    private const string IFEI_PAGE = "IFEI";
    private const string CALC_PAGE = "CALC";

    uint _masterCaution = 0;
    uint _lightMode = 0; // 2=NVG, 1=NITE, 0=DAY

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
        AddNewPage(CALC_PAGE);
    }

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        switch (_currentPage)
        {
            case CALC_PAGE:
                // Typing keys (digits, letters, CLR…) go to the scratchpad first.
                // If the scratchpad consumed the key there is nothing else to do.
                // Otherwise forward to the calc page (LSK commits, navigation away…).
                if (!Scratchpad.HandleKey(e.Key))
                    HandleCalcKey(e.Key);
                break;

            default:
                if (e.Key == Key.Perf)          { _currentPage = CALC_PAGE;   RenderCalcPage(); }
                else if (e.Key == _nextPageKey) { _currentPage = IFEI_PAGE;   RenderIfei(); }
                else if (e.Key == _prevPageKey) { _currentPage = DEFAULT_PAGE; RenderUfc(); }
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

        RegisterUInt("COCKKPIT_LIGHT_MODE_SW", v => _lightMode = v);
        RegisterUInt("MASTER_CAUTION_LT", v =>
        {
            if (_masterCaution != v)
            {
                _masterCaution = v;
                SetCduLeds(fail: _masterCaution != 0);
            }
        });

        RegisterUfcControls();
        RegisterIfeiControls();
        RegisterCalcControls();
    }

    protected override void RegisterFrontpanelControls()
    {
        RegisterUInt("FLP_LG_LEFT_GEAR_LT",    v => FlightDeck.GearLeftDown = v == 1);
        RegisterUInt("FLP_LG_RIGHT_GEAR_LT",   v => FlightDeck.GearRightDown = v == 1);
        RegisterUInt("FLP_LG_NOSE_GEAR_LT",    v => FlightDeck.GearNoseDown = v == 1);
        RegisterUInt("LANDING_GEAR_HANDLE_LT", v => FlightDeck.GearWarning = v == 1);
        RegisterUInt("FLP_LG_HALF_FLAPS_LT",   v => FlightDeck.LedAutoBrkLoDecel = v == 1);
        RegisterUInt("FLP_LG_FULL_FLAPS_LT",   v => FlightDeck.LedAutoBrkMedDecel = v == 1);

        RegisterIfeiFrontPanelControls();
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
