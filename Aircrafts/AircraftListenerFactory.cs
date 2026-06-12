using WWCduDcsBiosBridge.Devices.Cdu;

namespace WWCduDcsBiosBridge.Aircrafts;

internal interface IAircraftListenerFactory
{
    public AircraftListener CreateListener(AircraftSelection aircraft, AircraftCduContext? cduContext, UserOptions options, bool ch47SwitchWithSeat);
}


internal class AircraftListenerFactory : IAircraftListenerFactory
{
    public AircraftListener CreateListener(
        AircraftSelection aircraft,
        AircraftCduContext? cduContext,
        UserOptions options,
        bool ch47SwitchWithSeat)
    {
        var listener = AircraftRegistry.Find(aircraft.AircraftId)
            .Create(new AircraftCreationContext(options, aircraft.IsPilot, ch47SwitchWithSeat));

        if (cduContext != null)
        {
            listener.AttachCduContext(cduContext);
        }

        return listener;
    }
}
