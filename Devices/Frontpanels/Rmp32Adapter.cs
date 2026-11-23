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
        _device.ControlActivated += (_, e) => ControlActivated?.Invoke(e.ControlId);
    }

    /// <summary>
    /// Raised on the device's input thread when a button is pressed or a ring detent turns,
    /// with the name of the control as it appears in <c>WwDevicesDotNet.Winctrl.Rmp32.Control</c>.
    /// </summary>
    public event Action<string>? ControlActivated;

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
