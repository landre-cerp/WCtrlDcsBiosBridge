using System;
using System.Collections.Generic;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

/// <summary>
/// Translates the semantic <see cref="FlightDeckState"/> to one family of frontpanel
/// devices. A renderer owns its family's device state/led objects and its lighting
/// policy; it is created only when at least one adapter of its family is connected.
///
/// Displays and LEDs are rendered on separate cadences (see <see cref="FrontpanelHub"/>):
/// nobody reads an airspeed at 30 Hz, but a flashing caution lamp sampled at 10 Hz comes out
/// as a slower, uneven blink that does not match the one in the cockpit.
/// </summary>
internal abstract class FrontpanelRenderer
{
    protected readonly IReadOnlyList<IFrontpanelAdapter> adapters;
    protected readonly bool manageLighting;

    private (byte Panel, byte Lcd, byte Led)? _lastBrightness;
    private ulong? _lastLedMask;

    protected FrontpanelRenderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
    {
        this.adapters = adapters ?? throw new ArgumentNullException(nameof(adapters));
        this.manageLighting = manageLighting;
    }

    /// <summary>
    /// Pushes the display side of the model to every connected adapter of this family.
    /// Called from the hub's render timer (single thread).
    /// </summary>
    public abstract void RenderDisplay(FlightDeckState model);

    /// <summary>
    /// Pushes the LED side of the model. Runs far more often than
    /// <see cref="RenderDisplay"/> and is expected to send nothing most times it is called.
    /// </summary>
    public abstract void RenderLeds(FlightDeckState model);

    /// <summary>
    /// Forgets what was last sent, so the next render pushes everything again. Call after the
    /// devices have been reset behind the renderer's back: they are dark, while the caches
    /// below still describe the frame before the reset, and "unchanged" would keep them dark.
    /// </summary>
    public void Invalidate()
    {
        _lastBrightness = null;
        _lastLedMask = null;
    }

    /// <summary>
    /// Works out every LED of <paramref name="family"/> and writes it into
    /// <paramref name="leds"/>: the signal the LED shows by default, overridden by the user's
    /// binding where they made one. Returns the frame as a bitmask, one bit per catalog entry,
    /// for <see cref="LedsChanged"/> to compare against the frame before it.
    /// </summary>
    protected static ulong BuildLeds(FlightDeckState model, IFrontpanelLeds leds, LedDeviceFamily family)
    {
        var catalog = LedCatalog.For(family);
        var hasUserLeds = model.HasUserLeds;
        ulong mask = 0;

        for (var i = 0; i < catalog.Count; i++)
        {
            var led = catalog[i];
            if (led.Set == null) continue;

            var on = led.Signal is FlightDeckSignal signal && (model.GetSignal(signal) ?? false);

            // The user's binding is applied last, so it wins on the LEDs they bound and leaves
            // every other one to the behaviour the aircraft gives it.
            if (hasUserLeds && model.TryGetUserLed(family, led.Id, out var bound)) on = bound;

            led.Set(leds, on);
            if (on) mask |= 1UL << i;
        }

        return mask;
    }

    /// <summary>
    /// Whether this LED frame differs from the one last sent. Sending a frame costs one HID
    /// write per lamp on the panel — the PAP-3 sends seventeen — so at 30 Hz the only
    /// affordable frame is the one that changed something.
    /// </summary>
    protected bool LedsChanged(ulong mask)
    {
        if (_lastLedMask == mask) return false;
        _lastLedMask = mask;
        return true;
    }

    /// <summary>Pushes a LED frame to every connected adapter of this family.</summary>
    protected void SendLeds(IFrontpanelLeds leds)
    {
        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateLeds(leds);
        }
    }

    /// <summary>Pushes a display frame to every connected adapter of this family.</summary>
    protected void SendDisplay(IFrontpanelState state)
    {
        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(state);
        }
    }

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
