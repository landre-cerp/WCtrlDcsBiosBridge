using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Rmp32;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to RMP32 radio panels. No aircraft drives its LEDs or
/// displays on its own yet: the LEDs follow the user's bindings, the displays stay blank.
/// </summary>
internal class Rmp32Renderer : FrontpanelRenderer
{
    private readonly Rmp32State _state = new();

    private readonly Rmp32Leds _leds = new();

    public Rmp32Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void RenderDisplay(FlightDeckState model)
    {
        ApplyBrightness(model);
        SendDisplay(_state);
    }

    public override void RenderLeds(FlightDeckState model)
    {
        var mask = BuildLeds(model, _leds, LedDeviceFamily.Rmp32);
        if (LedsChanged(mask)) SendLeds(_leds);
    }
}
