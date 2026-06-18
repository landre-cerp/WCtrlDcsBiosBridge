using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter for AGP32 devices that implements the capability-based pattern.
/// </summary>
public class Agp32Adapter : IFrontpanelAdapter
{
    private readonly Agp32Device _device;

    public string DisplayName { get; }
    public bool IsConnected => _device.IsConnected;

    public IFrontpanelCapabilities Capabilities => _device.Capabilities;

    public Agp32Adapter(Agp32Device device, string displayName)
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
