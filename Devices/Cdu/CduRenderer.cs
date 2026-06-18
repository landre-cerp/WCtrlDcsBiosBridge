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
    public bool LedsDirty { get; set; } = true;

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

            if (state.LedsDirty)
            {
                _device.Leds.Fail = state.LedFail;
                _device.Leds.Fm1 = state.LedFm1;
                _device.Leds.Fm2 = state.LedFm2;
                _device.Leds.Fm = state.LedFm;
                _device.Leds.Ind = state.LedInd;
                _device.Leds.Rdy = state.LedRdy;
                _device.RefreshLeds();
                state.LedsDirty = false;
            }
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

    public void Reset() => Device.Reset();

    public void Render(Screen screen) => _renderer.Render(screen, State);

    public void Cleanup()
    {
        lock (State.SyncRoot)
        {
            _renderer.Cleanup();
        }
    }
}
