using System;
using System.Collections.Generic;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Translates the semantic <see cref="FlightDeckState"/> to one family of frontpanel
/// devices. A renderer owns its family's device state/led objects and its lighting
/// policy; it is created only when at least one adapter of its family is connected.
/// </summary>
internal abstract class FrontpanelRenderer
{
    protected readonly IReadOnlyList<IFrontpanelAdapter> adapters;
    protected readonly bool manageLighting;

    private (byte Panel, byte Lcd, byte Led)? _lastBrightness;

    protected FrontpanelRenderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
    {
        this.adapters = adapters ?? throw new ArgumentNullException(nameof(adapters));
        this.manageLighting = manageLighting;
    }

    /// <summary>
    /// Pushes the model to every connected adapter of this family.
    /// Called from the hub's render timer (single thread).
    /// </summary>
    public abstract void Render(FlightDeckState model);

    /// <summary>
    /// Sends brightness to the family's adapters, but only when it changed
    /// (SetBrightness is several HID commands per call). No-op when lighting
    /// management is disabled (SimAppPro users).
    /// </summary>
    protected void ApplyBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight)
    {
        if (!manageLighting) return;

        var target = (panelBacklight, lcdBacklight, ledBacklight);
        if (_lastBrightness == target) return;
        _lastBrightness = target;

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.SetBrightness(panelBacklight, lcdBacklight, ledBacklight);
        }
    }
}
