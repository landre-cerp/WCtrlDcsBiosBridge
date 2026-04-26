using WwDevicesDotNet;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge.Aircrafts;

internal interface IAircraftListenerFactory
{
    public AircraftListener CreateListener(AircraftSelection aircraft, ICdu? mcdu, UserOptions options, FrontpanelHub frontpanelHub);
}


internal class AircraftListenerFactory : IAircraftListenerFactory
{
    public AircraftListener CreateListener(
        AircraftSelection aircraft, 
        ICdu? mcdu, 
        UserOptions options,
        FrontpanelHub frontpanelHub) =>

        aircraft.AircraftId switch
        {
            SupportedAircrafts.A10C => new A10C_Listener(mcdu, options, frontpanelHub),
            SupportedAircrafts.AH64D => new AH64D_Listener(mcdu, options),
            SupportedAircrafts.FA18C => new FA18C_Listener(mcdu, options),
            SupportedAircrafts.CH47 => new CH47F_Listener(mcdu, options, aircraft.IsPilot),
            SupportedAircrafts.F15E => new F15E_Listener(mcdu, options),
            SupportedAircrafts.M2000C => new M2000C_Listener(mcdu, options),
            SupportedAircrafts.F16C  => new F16C_Listener(mcdu, options, frontpanelHub),
            _ => throw new NotSupportedException($"Aircraft {aircraft.AircraftId} not supported")

        };

}