using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.F14;

internal partial class F14_Listener : AircraftListener
{
    private const string RADIO_PAGE = "RADIO";

    private readonly Key _rioDisplayKey;
    private readonly Key _radioDisplayKey;


    private string _clockH  = "00";
    private string _clockM  = "00";
    private string _timerT  = "00";
    private string _timerTm = "00";
    private string _timerTs = "00";

    public F14_Listener(UserOptions options) : base(AircraftRegistry.F14B, options)
    {
        _rioDisplayKey = Enum.TryParse<Key>(options.F14.RioKey, out var rioKey)
            ? rioKey : Key.PrevPage;
        _radioDisplayKey = Enum.TryParse<Key>(options.F14.RadioKey, out var radioKey)
            ? radioKey : Key.NextPage;

        AddNewPage(RADIO_PAGE);
    }

    ~F14_Listener() => Dispose(false);

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == _rioDisplayKey)
        {
            _currentPage = DEFAULT_PAGE;
            RenderRioPage();
        }
        else if (e.Key == _radioDisplayKey)
        {
            _currentPage = RADIO_PAGE;
            RenderRadioPage();
        }
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        RegisterRioControls();
        RegisterRadioControls();
        _currentPage = RADIO_PAGE;
        RenderRadioPage();
    }

    protected override void RegisterFrontpanelControls()
    {
        // The gear lights, warning included, are declared in LedDefaults.

        RegisterUInt("PLT_CLOCK_H",  (ctrl, v) => { _clockH  = ((int)(v * 12 / (ctrl.MaxValue + 1))).ToString("D2"); FlightDeck.ClockUtcTime = _clockH + _clockM + "00"; });
        RegisterUInt("PLT_CLOCK_M",  (ctrl, v) => { _clockM  = ((int)(v * 60 / (ctrl.MaxValue + 1))).ToString("D2"); FlightDeck.ClockUtcTime = _clockH + _clockM + "00"; });
        RegisterUInt("PLT_CLOCK_T",  (ctrl, v) => { _timerT  = ((int)(v * 12 / (ctrl.MaxValue + 1))).ToString("D2"); UpdateChrono(); });
        RegisterUInt("PLT_CLOCK_TM", (ctrl, v) => { _timerTm = ((int)(v * 60 / (ctrl.MaxValue + 1))).ToString("D2"); UpdateChrono(); });
        RegisterUInt("PLT_CLOCK_TS", (ctrl, v) => { _timerTs = ((int)(v * 60 / (ctrl.MaxValue + 1))).ToString("D2"); UpdateChrono(); });
    }

    private void UpdateChrono()
    {
        FlightDeck.ClockChrono = _timerTm + _timerTs;
        FlightDeck.ClockElapsedTime = _timerT != "00" ? _timerT + _timerTm : string.Empty;
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && CduDevice != null)
            CduDevice.KeyDown -= HandleKeyDown;
        base.Dispose(disposing);
    }
}
