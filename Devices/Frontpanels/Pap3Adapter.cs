using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Pap3;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter for PAP3 devices that implements the capability-based pattern.
/// </summary>
public class Pap3Adapter : IFrontpanelAdapter
{
    private readonly Pap3Device _device;

    public string DisplayName { get; }
    public bool IsConnected => _device.IsConnected;

    public IFrontpanelCapabilities Capabilities => _device.Capabilities;

    public Pap3Adapter(Pap3Device device, string displayName)
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
