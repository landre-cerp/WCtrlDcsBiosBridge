using System;
using System.Collections.Generic;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

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

    protected void ApplyBrightness(
        FlightDeckState model,
        byte fallbackPanelBacklight = 127,
        byte fallbackLcdBacklight = 255,
        byte fallbackLedBacklight = 255)
    {
        if (model.ConsoleBrightness is byte b)
        {
            var segment = PercentToByte(model.SegmentBrightnessPercent);
            ApplyDeviceBrightness(b, segment, segment);
        }
        else
            ApplyDeviceBrightness(fallbackPanelBacklight, fallbackLcdBacklight, fallbackLedBacklight);
    }

    private static byte PercentToByte(int percent)
    {
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        return (byte)(percent * 255 / 100);
    }

    /// <summary>
    /// Sends brightness to the family's adapters, but only when it changed
    /// (SetBrightness is several HID commands per call). No-op when lighting
    /// management is disabled (SimAppPro users).
    /// </summary>
    private void ApplyDeviceBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight)
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
