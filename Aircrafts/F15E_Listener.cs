using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class F15E_Listener : AircraftListener
{
    public F15E_Listener(UserOptions options) : base(AircraftRegistry.F15E, options)
    {
    }

    protected override void RegisterCduControls()
    {
        RegisterStr("F_UFC_LINE1_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(2).White().Centered(s));
        RegisterStr("F_UFC_LINE2_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(4).Red().Centered(s));
        RegisterStr("F_UFC_LINE3_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(6).Red().Centered(s));
        RegisterStr("F_UFC_LINE4_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(8).Red().Centered(s));
        RegisterStr("F_UFC_LINE5_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(10).White().Centered(s));
        RegisterStr("F_UFC_LINE6_DISPLAY", s => GetCompositor(DEFAULT_PAGE).Line(12).White().Centered(s));
    }

    protected override void RegisterFrontpanelControls() { }
}
