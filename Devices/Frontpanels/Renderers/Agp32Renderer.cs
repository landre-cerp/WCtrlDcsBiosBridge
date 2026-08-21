using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Agp32;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

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

    public override void RenderDisplay(FlightDeckState model)
    {
        // Family policy: stay visible even when the cockpit console brightness
        // is at zero — the AGP32 gear lights are useless when dark.
        ApplyBrightness(model);

        _state.ChrDisplay = model.ClockChrono ?? string.Empty;
        _state.ClockDisplay = model.ClockUtcTime ?? string.Empty;
        _state.EtDisplay = model.ClockElapsedTime ?? string.Empty;

        SendDisplay(_state);
    }

    public override void RenderLeds(FlightDeckState model)
    {
        // Every LED on this panel names the signal it shows (see LedCatalog), so the gear and
        // autobrake mapping lives there rather than here. One signal can light several: a
        // single gear-in-transit warning drives the three red UNLK triangles and the lever's
        // red arrow together.
        var mask = BuildLeds(model, _leds, LedDeviceFamily.Agp32);
        if (LedsChanged(mask)) SendLeds(_leds);
    }
}
