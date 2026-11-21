using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Rmp32;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter for RMP32 devices that implements the capability-based pattern.
/// </summary>
public class Rmp32Adapter : IFrontpanelAdapter
{
    private readonly Rmp32Device _device;

    public string DisplayName { get; }
    public bool IsConnected => _device.IsConnected;

    public IFrontpanelCapabilities Capabilities => _device.Capabilities;

    public Rmp32Adapter(Rmp32Device device, string displayName)
    {
        _device = device ?? throw new ArgumentNullException(nameof(device));
        DisplayName = displayName ?? throw new ArgumentNullException(nameof(displayName));
    }

    public void Reset()
    {
        _device.Reset();
    }

    public void UpdateDisplay(IFrontpanelState state)
    {
        _device.UpdateDisplay(state);
    }

    public void UpdateLeds(IFrontpanelLeds leds)
    {
        _device.UpdateLeds(leds);
    }

    public void SetBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight)
    {
        _device.SetBrightness(panelBacklight, lcdBacklight, ledBacklight);
    }
}
