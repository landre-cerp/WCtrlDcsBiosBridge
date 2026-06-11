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

        AircraftRegistry.Find(aircraft.AircraftId)
            .Create(new AircraftCreationContext(mcdu, options, aircraft.IsPilot, ch47SwitchWithSeat));
}
