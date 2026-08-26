using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Devices.Cdu;

internal sealed class CduRenderState
{
    public object SyncRoot { get; } = new();

public int BacklightBrightnessPercent { get; set; } = 100;
public int DisplayBrightnessPercent { get; set; } = 100;
public int LedBrightnessPercent { get; set; } = 100;
public bool BrightnessDirty { get; set; } = false;

    public bool LedFail { get; set; }
    public bool LedFm1 { get; set; }
    public bool LedFm2 { get; set; }
    public bool LedFm { get; set; }
    public bool LedInd { get; set; }
    public bool LedRdy { get; set; }

    /// <summary>
    /// The PFP's EXEC annunciator. The MCDU has no such lamp and the PFPs have no RDY, so an
    /// aircraft with something to say about execution sets both and each panel lights the one
    /// it carries — writing a LED a panel does not have costs nothing, the device ignores it.
    /// </summary>
    public bool LedExec { get; set; }

    public bool LedsDirty { get; set; } = true;

    /// <summary>
    /// Forgets which annunciators were lit, after the device itself has been blanked. This
    /// state outlives the listener that wrote it — the same context serves the next aircraft —
    /// so leaving it claiming a lamp is lit while the hardware is dark would keep the next
    /// session from lighting it again when the same value comes back.
    /// </summary>
    public void ForgetLeds()
    {
        LedFail = LedFm1 = LedFm2 = LedFm = LedInd = LedRdy = LedExec = false;
        LedsDirty = true;
    }

    public McduFontFile? Font { get; set; }
    public bool FontDirty { get; set; }
}

internal sealed class CduRenderer
{
    private readonly ICdu _device;

    public CduRenderer(ICdu device)
    {
        _device = device;
    }

    /// <summary>
    /// Pushes the annunciators, if any of them moved. Runs on its own tick, far more often
    /// than the screen: a flashing caution lamp sampled at 10 Hz blinks visibly slower than
    /// the one in the cockpit, and this costs nothing on the ticks where nothing changed.
    /// </summary>
    public void RenderLeds(CduRenderState state)
    {
        lock (state.SyncRoot)
        {
            if (!state.LedsDirty) return;

            _device.Leds.Fail = state.LedFail;
            _device.Leds.Fm1 = state.LedFm1;
            _device.Leds.Fm2 = state.LedFm2;
            _device.Leds.Fm = state.LedFm;
            _device.Leds.Ind = state.LedInd;
            _device.Leds.Rdy = state.LedRdy;
            _device.Leds.Exec = state.LedExec;
            _device.RefreshLeds();
            state.LedsDirty = false;
        }
    }

    public void Render(Screen source, CduRenderState state)
    {
        lock (state.SyncRoot)
        {
            if (state.FontDirty && state.Font != null)
            {
                _device.UseFont(state.Font, true);
                state.FontDirty = false;
            }

            _device.Screen.CopyFrom(source);
            _device.RefreshDisplay();

            if (state.BrightnessDirty)
            {
                _device.BacklightBrightnessPercent = state.BacklightBrightnessPercent;
                _device.DisplayBrightnessPercent = state.DisplayBrightnessPercent;
                _device.LedBrightnessPercent = state.LedBrightnessPercent;
                _device.RefreshBrightnesses();
                state.BrightnessDirty = false;
            }

            RenderLeds(state);
        }
    }

    public void Cleanup()
    {
        _device.Output.Clear();
        _device.Cleanup();
        _device.RefreshDisplay();
    }
}

internal sealed class AircraftCduContext
{
    public ICdu Device { get; }
    public CduRenderState State { get; } = new();

    private readonly CduRenderer _renderer;

    public AircraftCduContext(ICdu device)
    {
        Device = device;
        _renderer = new CduRenderer(device);
    }

    public event EventHandler<KeyEventArgs> KeyDown
    {
        add => Device.KeyDown += value;
        remove => Device.KeyDown -= value;
    }

    public void Reset()
    {
        Device.Reset();
        lock (State.SyncRoot)
        {
            State.ForgetLeds();
        }
    }

    public void Render(Screen screen) => _renderer.Render(screen, State);

    /// <summary>Pushes the annunciators alone, on the LED tick.</summary>
    public void RenderLeds() => _renderer.RenderLeds(State);

    public void Cleanup()
    {
        lock (State.SyncRoot)
        {
            _renderer.Cleanup();
            State.ForgetLeds();
        }
    }
}
