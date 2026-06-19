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
