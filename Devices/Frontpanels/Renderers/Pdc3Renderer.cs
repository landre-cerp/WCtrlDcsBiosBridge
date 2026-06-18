using System.Collections.Generic;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

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
        ApplyBrightness(model);
    }
}
