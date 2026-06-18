using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Devices;

/// <summary>
/// Contains information about a detected CDU or Frontpanel device
/// </summary>
public class DeviceInfo
{
    private readonly object _device;

    public ICdu? Cdu => _device as ICdu;
    public IFrontpanel? Frontpanel => _device as IFrontpanel;
    public DeviceIdentifier DeviceId { get; set; }
    public string DisplayName { get; set; }

    public DeviceInfo(ICdu cdu, DeviceIdentifier deviceId, string displayName)
    {
        _device = cdu;
        DeviceId = deviceId;
        DisplayName = displayName;
    }

    public DeviceInfo(IFrontpanel frontpanel, DeviceIdentifier deviceId, string displayName)
    {
        _device = frontpanel;
        DeviceId = deviceId;
        DisplayName = displayName;
    }

    public void Dispose()
    {
        (Cdu as IDisposable)?.Dispose();
        (Frontpanel as IDisposable)?.Dispose();
    }
}
