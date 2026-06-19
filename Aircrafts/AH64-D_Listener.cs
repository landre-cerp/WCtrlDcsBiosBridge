using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class AH64D_Listener : AircraftListener
{
    public AH64D_Listener(UserOptions options) : base(AircraftRegistry.AH64D, options) {
    }

    ~AH64D_Listener()
    {
        Dispose(false);
    }

    protected override void RegisterCduControls()
    {
        if (!options.DisableLightingManagement && HasCdu)
        {
            RegisterUInt("PLT_EUFD_BRT", v =>
            {
                int eufdBright = 100 * (int)v / 65536;
                SetBacklightBrightnessPercent(eufdBright);
                SetDisplayBrightnessPercent(eufdBright);
                SetLedBrightnessPercent(eufdBright);
            });
        }

        // Note that they share the same Address but bit is different (10 and 11)
        RegisterUInt("PLT_MASTER_CAUTION_L", v => SetCduLeds(fail: v == 1));
        RegisterUInt("PLT_MASTER_WARNING_L", v => SetCduLeds(ind: v == 1));

        RegisterStr("PLT_EUFD_LINE14", s =>
        {
            var data = NormalizeEufdString(s);
            GetCompositor(DEFAULT_PAGE).Line(0).Green().WriteLine($"{data.Substring(0, 10)}    {data.Substring(46, 10)}");
        });

        RegisterStr("PLT_EUFD_LINE1",  s => GetCompositor(DEFAULT_PAGE).Line(1).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterStr("PLT_EUFD_LINE2",  s => GetCompositor(DEFAULT_PAGE).Line(2).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterStr("PLT_EUFD_LINE3",  s => GetCompositor(DEFAULT_PAGE).Line(3).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterStr("PLT_EUFD_LINE4",  s => GetCompositor(DEFAULT_PAGE).Line(4).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterStr("PLT_EUFD_LINE5",  s => GetCompositor(DEFAULT_PAGE).Line(5).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));

        RegisterStr("PLT_EUFD_LINE8",  s => GetCompositor(DEFAULT_PAGE).Line(7).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterStr("PLT_EUFD_LINE9",  s => GetCompositor(DEFAULT_PAGE).Line(8).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterStr("PLT_EUFD_LINE10", s => GetCompositor(DEFAULT_PAGE).Line(9).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterStr("PLT_EUFD_LINE11", s => GetCompositor(DEFAULT_PAGE).Line(10).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterStr("PLT_EUFD_LINE12", s => GetCompositor(DEFAULT_PAGE).Line(11).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));

        GetCompositor(DEFAULT_PAGE).Line(12).Amber().WriteLine("- Keyboard -------------");

        RegisterStr("PLT_KU_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(13).Green().WriteLine(NormalizeEufdString(s)));
    }

    protected override void RegisterFrontpanelControls() { }

    private static string NormalizeEufdString(string s) =>
        s.Replace("~", "█")
         .Replace(">", "▶")
         .Replace("<", "◀")
         .Replace("=", "■")
         .Replace("#", "█")
         .PadRight(60)
         [..60];
}
