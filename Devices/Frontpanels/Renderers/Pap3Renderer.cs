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

    public override void Render(FlightDeckState model)
    {
        ApplyBrightness(model);

        _state.Speed = model.Speed;
        _state.Heading = model.Heading;
        _state.Altitude = model.Altitude;
        _state.VerticalSpeed = model.VerticalSpeed;
        _state.PltCourseValue = model.PltCourse;
        _state.CplCourseValue = model.CplCourse;

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(_leds);
        }
    }
}
