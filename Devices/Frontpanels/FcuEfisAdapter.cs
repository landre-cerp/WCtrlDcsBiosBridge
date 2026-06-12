using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.FcuAndEfis;

namespace WWCduDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter for FCU/EFIS devices that implements the capability-based pattern.
/// </summary>
public class FcuEfisAdapter : IFrontpanelAdapter
{
    private readonly FcuEfisDevice _device;

    public IFrontpanel Device => _device;
    public string DisplayName { get; }
    public bool IsConnected => _device.IsConnected;

    public IFrontpanelCapabilities Capabilities => _device.Capabilities;

    public FcuEfisAdapter(FcuEfisDevice device, string displayName)
    {
        _device = device ?? throw new ArgumentNullException(nameof(device));
        DisplayName = displayName ?? throw new ArgumentNullException(nameof(displayName));
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
