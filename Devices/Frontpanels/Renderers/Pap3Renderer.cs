using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Pap3;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to PAP3 devices (Boeing 737 style).
/// </summary>
internal class Pap3Renderer : FrontpanelRenderer
{
    private readonly Pap3State _state = new();

    private readonly Pap3Leds _leds = new();

    public Pap3Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void RenderDisplay(FlightDeckState model)
    {
        ApplyBrightness(model);

        _state.Speed = model.Speed;
        _state.Heading = model.Heading;
        _state.Altitude = model.Altitude;
        _state.VerticalSpeed = model.VerticalSpeed;
        _state.PltCourseValue = model.PltCourse;
        _state.CplCourseValue = model.CplCourse;

        SendDisplay(_state);
    }

    public override void RenderLeds(FlightDeckState model)
    {
        // The Boeing autopilot legends have no semantic equivalent — nothing in a DCS module
        // means "CMD A" — so every lit LED here comes from a user binding.
        var mask = BuildLeds(model, _leds, LedDeviceFamily.Pap3);
        if (LedsChanged(mask)) SendLeds(_leds);
    }
}
