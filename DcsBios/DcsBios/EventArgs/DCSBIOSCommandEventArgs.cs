namespace DCS_BIOS.EventArgs
{
    public class DCSBIOSCommandEventArgs : System.EventArgs                 
    {
        public string Sender { get; init; }
        public string Command { get; init; }
    }
}
