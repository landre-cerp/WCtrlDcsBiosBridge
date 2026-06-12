using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Pap3;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to PAP3 devices (Boeing 737 style).
/// </summary>
internal class Pap3Renderer : FrontpanelRenderer
{
    // PAP3 LCDs turn off on empty/null values, so default to zeros.
    private readonly Pap3State _state = new()
    {
        Speed = 0,
        Heading = 0,
        Altitude = 0,
        VerticalSpeed = 0,
    };

    private readonly Pap3Leds _leds = new();

    public Pap3Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        ApplyBrightnessFromConsoleAndSegmentPercent(model);

        _state.Speed = model.Speed ?? 0;
        _state.Heading = model.Heading ?? 0;
        _state.Altitude = model.Altitude ?? 0;
        _state.VerticalSpeed = model.VerticalSpeed ?? 0;

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(_leds);
        }
    }
}
