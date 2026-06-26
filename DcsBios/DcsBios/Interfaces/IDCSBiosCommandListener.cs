namespace DCS_BIOS.Interfaces
{
    using EventArgs;

    public interface IDCSBiosCommandListener
    {
        void DCSBIOSCommandSent(DCSBIOSCommandEventArgs e);
    }
}
