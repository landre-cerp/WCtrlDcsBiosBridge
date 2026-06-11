using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Agp32;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to AGP32 clock/gear panels (A320 style).
/// </summary>
internal class Agp32Renderer : FrontpanelRenderer
{
    private readonly Agp32State _state = new();
    private readonly Agp32State.Agp32Leds _leds = new();

    public Agp32Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        // Family policy: stay visible even when the cockpit console brightness
        // is at zero — the AGP32 gear lights are useless when dark.
        ApplyBrightness(255, 255, 255);

        if (model.GearLeftDown is bool left)
            _leds.Set(Agp32State.Agp32Led.Gear1Down, left);
        if (model.GearNoseDown is bool nose)
            _leds.Set(Agp32State.Agp32Led.Gear2Down, nose);
        if (model.GearRightDown is bool right)
            _leds.Set(Agp32State.Agp32Led.Gear3Down, right);
        if (model.GearWarning is bool warning)
            _leds.Set(Agp32State.Agp32Led.GearDownRed, warning);

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(_leds);
        }
    }
}
