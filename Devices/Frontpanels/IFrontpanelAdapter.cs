using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Adapter interface that exposes capabilities of a frontpanel device
/// in a capability-based pattern (display update, LED update, brightness control).
/// </summary>
public interface IFrontpanelAdapter
{
    IFrontpanelCapabilities Capabilities { get; }

    /// <summary>
    /// Gets the device display name.
    /// </summary>
    string DisplayName { get; }

    /// <summary>
    /// Updates the display on the frontpanel.
    /// </summary>
    void UpdateDisplay(IFrontpanelState state);

    /// <summary>
    /// Updates the LEDs on the frontpanel.
    /// </summary>
    void UpdateLeds(IFrontpanelLeds leds);

    /// <summary>
    /// Sets the brightness levels for the frontpanel.
    /// </summary>
    void SetBrightness(byte panelBacklight, byte lcdBacklight, byte ledBacklight);

    /// <summary>
    /// Resets the frontpanel device to its default state.
    /// </summary>
    void Reset();

    /// <summary>
    /// Gets a value indicating whether the device is currently connected.
    /// </summary>
    bool IsConnected { get; }
}
