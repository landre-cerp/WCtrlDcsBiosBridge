using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to FCU/EFIS devices (Airbus style).
/// </summary>
internal class FcuEfisRenderer : FrontpanelRenderer
{
    private const byte DefaultPanelBacklight = 128;

    private readonly FcuEfisState _state = new();
    private readonly FcuEfisLeds _leds = new();

    public FcuEfisRenderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        if (model.ConsoleBrightness is byte b)
            ApplyBrightness(b, b, b);
        else
            ApplyBrightness(DefaultPanelBacklight, 255, 255);

        _state.Speed = model.Speed;
        _state.Heading = model.Heading;
        _state.Altitude = model.Altitude;
        _state.VerticalSpeed = model.VerticalSpeed;
        _state.LeftBaroPressure = model.BaroPressure;

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(_leds);
        }
    }
}
