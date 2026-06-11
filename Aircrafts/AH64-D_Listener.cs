using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class AH64D_Listener : AircraftListener
{
    // EUFD Display
    private DCSBIOSOutput? _PLT_EUFD_LINE1;
    private DCSBIOSOutput? _PLT_EUFD_LINE2;
    private DCSBIOSOutput? _PLT_EUFD_LINE3;
    private DCSBIOSOutput? _PLT_EUFD_LINE4;
    private DCSBIOSOutput? _PLT_EUFD_LINE5;

    private DCSBIOSOutput? _PLT_EUFD_LINE8;
    private DCSBIOSOutput? _PLT_EUFD_LINE9;
    private DCSBIOSOutput? _PLT_EUFD_LINE10;
    private DCSBIOSOutput? _PLT_EUFD_LINE11;
    private DCSBIOSOutput? _PLT_EUFD_LINE12;
    private DCSBIOSOutput? _PLT_EUFD_LINE14;

    // Keyboard display
    private DCSBIOSOutput? _PLT_KU_DISPLAY;

    // Brightness
    private DCSBIOSOutput? _PLT_EUFD_BRT;

    // Lights
    private DCSBIOSOutput? _PLT_MASTER_CAUTION_L;
    private DCSBIOSOutput? _PLT_MASTER_WARNING_L;
    

    public AH64D_Listener(ICdu? mcdu, UserOptions options) : base(mcdu, AircraftRegistry.AH64D, options) {
    }

    ~AH64D_Listener()
    {
        Dispose(false);
    }

    protected override void RegisterMcduControls()
    {
        if (!options.DisableLightingManagement && mcdu != null)
        {
            Register(_PLT_EUFD_BRT, v =>
            {
                int eufdBright = 100 * (int)v / 65536;
                mcdu!.BacklightBrightnessPercent = eufdBright;
                mcdu!.DisplayBrightnessPercent = eufdBright;
                mcdu!.LedBrightnessPercent = eufdBright;
                mcdu!.RefreshBrightnesses();
            });
        }

        Register(_PLT_MASTER_CAUTION_L, v => { mcdu!.Leds.Fail = v == 1; mcdu!.RefreshLeds(); });
        Register(_PLT_MASTER_WARNING_L, v => { mcdu!.Leds.Ind  = v == 1; mcdu!.RefreshLeds(); });

        RegisterString(_PLT_EUFD_LINE14, s =>
        {
            var data = NormalizeEufdString(s);
            GetCompositor(DEFAULT_PAGE).Line(0).Green().WriteLine($"{data.Substring(0, 10)}    {data.Substring(46, 10)}");
        });

        RegisterString(_PLT_EUFD_LINE1,  s => GetCompositor(DEFAULT_PAGE).Line(1).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterString(_PLT_EUFD_LINE2,  s => GetCompositor(DEFAULT_PAGE).Line(2).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterString(_PLT_EUFD_LINE3,  s => GetCompositor(DEFAULT_PAGE).Line(3).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterString(_PLT_EUFD_LINE4,  s => GetCompositor(DEFAULT_PAGE).Line(4).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));
        RegisterString(_PLT_EUFD_LINE5,  s => GetCompositor(DEFAULT_PAGE).Line(5).Green().WriteLine(NormalizeEufdString(s).Substring(38, 17)));

        RegisterString(_PLT_EUFD_LINE8,  s => GetCompositor(DEFAULT_PAGE).Line(7).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterString(_PLT_EUFD_LINE9,  s => GetCompositor(DEFAULT_PAGE).Line(8).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterString(_PLT_EUFD_LINE10, s => GetCompositor(DEFAULT_PAGE).Line(9).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterString(_PLT_EUFD_LINE11, s => GetCompositor(DEFAULT_PAGE).Line(10).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));
        RegisterString(_PLT_EUFD_LINE12, s => GetCompositor(DEFAULT_PAGE).Line(11).Green().WriteLine(NormalizeEufdString(s).Substring(0, 18)));

        GetCompositor(DEFAULT_PAGE).Line(12).Amber().WriteLine("- Keyboard -------------");

        RegisterString(_PLT_KU_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(13).Green().WriteLine(NormalizeEufdString(s)));
    }

    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        // PLT Keyboard display
        _PLT_KU_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_KU_DISPLAY");
        _PLT_EUFD_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_EUFD_BRT");

        // UFD Upper status 
        _PLT_EUFD_LINE1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE1");
        _PLT_EUFD_LINE2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE2");
        _PLT_EUFD_LINE3 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE3");
        _PLT_EUFD_LINE4 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE4");
        _PLT_EUFD_LINE5 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE5");

        // UFD Frequency
        _PLT_EUFD_LINE8 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE8");
        _PLT_EUFD_LINE9 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE9");
        _PLT_EUFD_LINE10 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE10");
        _PLT_EUFD_LINE11 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE11");
        _PLT_EUFD_LINE12 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE12");
        _PLT_EUFD_LINE14 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PLT_EUFD_LINE14");

        // Note that they share the same Address but bit is different ! (10 and 11 ) 
        _PLT_MASTER_CAUTION_L = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_MASTER_CAUTION_L");
        _PLT_MASTER_WARNING_L = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_MASTER_WARNING_L");
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
