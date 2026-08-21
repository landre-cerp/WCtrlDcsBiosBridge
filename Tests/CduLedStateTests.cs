using WCtrlDcsBiosBridge.Devices.Cdu;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// The CDU render state outlives the listener that writes it — one context serves every
/// aircraft the panel is used for — and LEDs are only pushed to the device when the state
/// says they changed. So blanking the hardware has to blank the state with it.
/// </summary>
public class CduLedStateTests
{
    [Fact]
    public void ForgetLeds_ClearsEveryAnnunciatorAndAsksForAPush()
    {
        var state = new CduRenderState
        {
            LedFail = true,
            LedFm1 = true,
            LedFm2 = true,
            LedFm = true,
            LedInd = true,
            LedRdy = true,
            LedsDirty = false,
        };

        state.ForgetLeds();

        Assert.False(state.LedFail);
        Assert.False(state.LedFm1);
        Assert.False(state.LedFm2);
        Assert.False(state.LedFm);
        Assert.False(state.LedInd);
        Assert.False(state.LedRdy);

        // Without this, a device blanked behind the state's back stays blank until some value
        // happens to change.
        Assert.True(state.LedsDirty);
    }

    [Fact]
    public void AFreshStateAsksForAPush() =>
        Assert.True(new CduRenderState().LedsDirty);
}
