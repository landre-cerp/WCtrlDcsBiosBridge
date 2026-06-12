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

    public Agp32Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        // Family policy: stay visible even when the cockpit console brightness
        // is at zero — the AGP32 gear lights are useless when dark.
        ApplyBrightness(model);

        _state.ChrDisplay = model.Agp32Chrono ?? string.Empty;
        _state.ClockDisplay = model.Agp32UtcTime ?? string.Empty;
        _state.EtDisplay = model.Agp32Et ?? string.Empty;

        var leds = new Agp32State.Agp32Leds();

        var left = model.GearLeftDown ?? false;
        var nose = model.GearNoseDown ?? false;
        var right = model.GearRightDown ?? false;
        var warning = model.GearWarning ?? false;

        leds.Set(Agp32State.Agp32Led.Gear1Down, left);
        leds.Set(Agp32State.Agp32Led.Gear2Down, nose);
        leds.Set(Agp32State.Agp32Led.Gear3Down, right);

        // A single gear-in-transit warning maps to the A320 panel's red UNLK
        // triangles (lit while a gear disagrees with the lever) plus the
        // lever's red arrow.
        leds.Set(Agp32State.Agp32Led.Gear1Unlk, warning);
        leds.Set(Agp32State.Agp32Led.Gear2Unlk, warning);
        leds.Set(Agp32State.Agp32Led.Gear3Unlk, warning);
        leds.Set(Agp32State.Agp32Led.GearDownRed, warning);

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            adapter.UpdateLeds(leds);
        }
    }
}
