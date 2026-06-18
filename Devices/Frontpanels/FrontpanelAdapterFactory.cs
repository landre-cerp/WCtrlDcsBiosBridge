using System;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;
using WwDevicesDotNet.Winctrl.Pdc3nm;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Factory for creating frontpanel adapters from IFrontpanel devices.
/// </summary>
public static class FrontpanelAdapterFactory
{
    /// <summary>
    /// Creates an adapter for the given frontpanel device.
    /// </summary>
    /// <param name="frontpanel">The frontpanel device.</param>
    /// <param name="displayName">Display name for the device.</param>
    /// <returns>An adapter for the device, or null if the device type is not supported.</returns>
    public static IFrontpanelAdapter? CreateAdapter(IFrontpanel frontpanel, string displayName)
    {
        if (frontpanel == null)
            throw new ArgumentNullException(nameof(frontpanel));

        return frontpanel switch
        {
            FcuEfisDevice fcuDevice => new FcuEfisAdapter(fcuDevice, displayName),
            Pap3Device pap3Device => new Pap3Adapter(pap3Device, displayName),
            Pdc3Device pdc3Device => new Pdc3Adapter(pdc3Device, displayName),
            Agp32Device agp32Device => new Agp32Adapter(agp32Device, displayName),
            _ => null
        };
    }
}
