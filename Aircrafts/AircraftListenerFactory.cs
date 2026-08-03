using WCtrlDcsBiosBridge.Devices.Cdu;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal interface IAircraftListenerFactory
{
    public AircraftListener CreateListener(AircraftSelection aircraft, AircraftCduContext? cduContext, UserOptions options, bool switchWithSeat);
}


internal class AircraftListenerFactory : IAircraftListenerFactory
{
    public AircraftListener CreateListener(
        AircraftSelection aircraft,
        AircraftCduContext? cduContext,
        UserOptions options,
        bool switchWithSeat)
    {
        var listener = AircraftRegistry.Find(aircraft.AircraftId)
            .Create(new AircraftCreationContext(options, aircraft.IsPilot, switchWithSeat));

        if (cduContext != null)
        {
            listener.AttachCduContext(cduContext);
        }

        return listener;
    }
}
