using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class AH64D_Listener : AircraftListener
{
    private const string CPG_PAGE = "CPG";
    private const int PLT_SEAT = 0;
    private const int CPG_SEAT = 1;

    private readonly bool _switchWithSeat;
    private int _seatPosition;

    public AH64D_Listener(UserOptions options, bool isPilot = true, bool switchWithSeat = false) : base(AircraftRegistry.AH64D, options)
    {
        _switchWithSeat = switchWithSeat;
        _seatPosition = isPilot ? PLT_SEAT : CPG_SEAT;

        // Always create the CPG page so CPG data is tracked regardless of active seat.
        AddNewPage(CPG_PAGE);

        // A dedicated CPG device always shows the CPG page.
        if (_seatPosition == CPG_SEAT)
            _currentPage = CPG_PAGE;
    }

    ~AH64D_Listener()
    {
        Dispose(false);
    }

    protected override void RegisterCduControls()
    {
        // --- Brightness: only the active seat drives the physical device ---
        RegisterLight("PLT_EUFD_BRT", v => { if (_seatPosition == PLT_SEAT) ApplyBrightness((int)v); });
        RegisterLight("CPG_EUFD_BRT", v => { if (_seatPosition == CPG_SEAT) ApplyBrightness((int)v); });

        // --- LEDs (PLT) ---
        // Note that they share the same Address but bit is different (10 and 11)
        RegisterUInt("PLT_MASTER_CAUTION_L", v => SetCduLeds(fail: v == 1));
        RegisterUInt("PLT_MASTER_WARNING_L", v => SetCduLeds(ind: v == 1));

        // --- PLT EUFD lines → DEFAULT_PAGE ---
        RegisterStr("PLT_EUFD_LINE14", s =>
        {
            var data = NormalizeEufdString(s);
            GetCompositor(DEFAULT_PAGE).Line(0).Green().WriteLine($"{data.Substring(0, 10)}    {data.Substring(46, 10)}");
        });

        for (int i = 1; i <= 5; i++)
        {
            int line = i;
            RegisterStr($"PLT_EUFD_LINE{line}", s => GetCompositor(DEFAULT_PAGE).Line(line).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        }

        for (int i = 8; i <= 12; i++)
        {
            int line = i;
            RegisterStr($"PLT_EUFD_LINE{line}", s => GetCompositor(DEFAULT_PAGE).Line(line - 1).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        }

        GetCompositor(DEFAULT_PAGE).Line(12).Amber().WriteLine("- Keyboard -------------");
        RegisterStr("PLT_KU_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(13).Green().WriteLine(NormalizeEufdString(s)));

        // --- CPG EUFD lines → CPG_PAGE (always registered so CPG data is tracked regardless of active seat) ---
        RegisterStr("CPG_EUFD_LINE14", s =>
        {
            var data = NormalizeEufdString(s);
            GetCompositor(CPG_PAGE).Line(0).Green().WriteLine($"{data.Substring(0, 10)}    {data.Substring(46, 10)}");
        });

        for (int i = 1; i <= 5; i++)
        {
            int line = i;
            RegisterStr($"CPG_EUFD_LINE{line}", s => GetCompositor(CPG_PAGE).Line(line).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        }

        for (int i = 8; i <= 12; i++)
        {
            int line = i;
            RegisterStr($"CPG_EUFD_LINE{line}", s => GetCompositor(CPG_PAGE).Line(line - 1).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        }

        GetCompositor(CPG_PAGE).Line(12).Amber().WriteLine("- Keyboard -------------");
        RegisterStr("CPG_KU_DISPLAY", s => GetCompositor(CPG_PAGE).Line(13).Green().WriteLine(NormalizeEufdString(s)));

        // --- Seat-position switching ---
        if (_switchWithSeat)
        {
            RegisterUInt("SEAT_POSITION", v =>
            {
                _seatPosition = (int)v;
                _currentPage = _seatPosition == PLT_SEAT ? DEFAULT_PAGE : CPG_PAGE;
            });
        }
    }

    protected override void RegisterFrontpanelControls() { }

    private void ApplyBrightness(int rawValue)
    {
        int bright = 100 * rawValue / 65536;
        SetCduBacklightBrightnessPercent(bright);
        SetCduDisplayBrightnessPercent(bright);
        SetCduLedBrightnessPercent(bright);
    }

    private static string NormalizeEufdString(string s) =>
        s.Replace("~", "█")
         .Replace(">", "▶")
         .Replace("<", "◀")
         .Replace("=", "■")
         .Replace("#", "█")
         .PadRight(60)
         [..60];
}
