using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class F15E_Listener : AircraftListener
{
    private DCSBIOSOutput? _F_UFC_LINE1_DISPLAY;
    private DCSBIOSOutput? _F_UFC_LINE2_DISPLAY;
    private DCSBIOSOutput? _F_UFC_LINE3_DISPLAY;
    private DCSBIOSOutput? _F_UFC_LINE4_DISPLAY;
    private DCSBIOSOutput? _F_UFC_LINE5_DISPLAY;
    private DCSBIOSOutput? _F_UFC_LINE6_DISPLAY;

    public F15E_Listener(UserOptions options) : base(AircraftRegistry.F15E, options)
    {
    }

    protected override void RegisterCduControls()
    {
        RegisterString(_F_UFC_LINE1_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(2).White().Centered(s));
        RegisterString(_F_UFC_LINE2_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(4).Red().Centered(s));
        RegisterString(_F_UFC_LINE3_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(6).Red().Centered(s));
        RegisterString(_F_UFC_LINE4_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(8).Red().Centered(s));
        RegisterString(_F_UFC_LINE5_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(10).White().Centered(s));
        RegisterString(_F_UFC_LINE6_DISPLAY, s => GetCompositor(DEFAULT_PAGE).Line(12).White().Centered(s));
    }

    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        _F_UFC_LINE1_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE1_DISPLAY");
        _F_UFC_LINE2_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE2_DISPLAY");
        _F_UFC_LINE3_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE3_DISPLAY");
        _F_UFC_LINE4_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE4_DISPLAY");
        _F_UFC_LINE5_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE5_DISPLAY");
        _F_UFC_LINE6_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("F_UFC_LINE6_DISPLAY");
    }
}
