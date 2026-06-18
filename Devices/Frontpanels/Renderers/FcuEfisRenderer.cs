using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to FCU/EFIS devices (Airbus style).
/// </summary>
internal class FcuEfisRenderer : FrontpanelRenderer
{
    private readonly FcuEfisState _state = new()
    {
        LeftBaroQnh = true,
        RightBaroQnh = true,
    };
    private readonly FcuEfisLeds _leds = new();

    public FcuEfisRenderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
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
        _state.LeftBaroPressure = model.BaroPressure;
        _state.RightBaroPressure = model.BaroPressure;

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(_leds);
        }
    }
}
