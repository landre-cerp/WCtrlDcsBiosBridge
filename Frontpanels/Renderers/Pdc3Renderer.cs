using System.Collections.Generic;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Renders to PDC-3N panel display controllers: brightness control only,
/// no display or LEDs (used alongside the PAP3 in Boeing 737 setups).
/// </summary>
internal class Pdc3Renderer : FrontpanelRenderer
{
    public Pdc3Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        if (model.ConsoleBrightness is byte b)
            ApplyBrightness(b, b, b);
        else
            ApplyBrightness(255, 255, 255);
    }
}
