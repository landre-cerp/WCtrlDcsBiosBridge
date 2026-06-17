using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Pdc3nm;

namespace WWCduDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter for WinWing PDC-3N devices.
/// PDC-3N is a panel controller device with only brightness management capability.
/// It has no display or LED features.
/// </summary>
public class Pdc3Adapter : IFrontpanelAdapter
{
    private readonly Pdc3Device _device;

    public string DisplayName { get; }
    public bool IsConnected => _device.IsConnected;

    public IFrontpanelCapabilities Capabilities => _device.Capabilities;

    public Pdc3Adapter(Pdc3Device device, string displayName)
    {
        _device = device ?? throw new ArgumentNullException(nameof(device));
        DisplayName = displayName ?? "PDC-3N";
    }

    public void Reset()
    {
        if (!IsConnected)
            return;

        _device.Reset();
    }

    /// <summary>
    /// Updates the display. PDC-3N has no display, so this is a no-op.
    /// </summary>
    public void UpdateDisplay(IFrontpanelState state)
    {
        // PDC-3N has no display capability
    }

    /// <summary>
    /// Updates the LEDs. PDC-3N has no LEDs, so this is a no-op.
    /// </summary>
    public void UpdateLeds(IFrontpanelLeds leds)
    {
        // PDC-3N has no LED capability
    }

    /// <summary>
    /// Sets the panel backlight brightness.
    /// PDC-3N only supports panel backlight control; lcdBacklight and ledBacklight are ignored.
    /// </summary>
    public void SetBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight)
    {
        if (!IsConnected)
            return;

        _device.SetBrightness(panelBacklight, lcdBacklight, ledBacklight);
    }
}
