using DCS_BIOS.Interfaces;

namespace WCtrlDcsBiosBridge;

internal interface IDcsBiosListener : IDcsBiosConnectionListener , IDcsBiosDataListener, IDCSBIOSStringListener
{
    public void Start();

    public void Stop();
}
