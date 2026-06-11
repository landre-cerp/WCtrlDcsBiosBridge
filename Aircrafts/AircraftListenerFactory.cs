using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal interface IAircraftListenerFactory
{
    public AircraftListener CreateListener(AircraftSelection aircraft, ICdu? mcdu, UserOptions options, bool ch47SwitchWithSeat);
}


internal class AircraftListenerFactory : IAircraftListenerFactory
{
    public AircraftListener CreateListener(
        AircraftSelection aircraft,
        ICdu? mcdu,
        UserOptions options,
        bool ch47SwitchWithSeat) =>

        aircraft.AircraftId switch
        {
            SupportedAircrafts.A10C => new A10C_Listener(mcdu, options),
            SupportedAircrafts.AH64D => new AH64D_Listener(mcdu, options),
            SupportedAircrafts.FA18C => new FA18C_Listener(mcdu, options),
            SupportedAircrafts.CH47 => new CH47F_Listener(mcdu, options, aircraft.IsPilot, ch47SwitchWithSeat),
            SupportedAircrafts.OH58D => new OH58D_Listener(mcdu, options),
            SupportedAircrafts.F15E => new F15E_Listener(mcdu, options),
            SupportedAircrafts.M2000C => new M2000C_Listener(mcdu, options),
            SupportedAircrafts.F16C  => new F16C_Listener(mcdu, options),
            _ => throw new NotSupportedException($"Aircraft {aircraft.AircraftId} not supported")

        };

}