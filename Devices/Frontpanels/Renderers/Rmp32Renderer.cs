using System;
using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Rmp32;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to RMP32 radio panels. The two displays show the model's
/// radio panel (active on the left, standby on the right) and stay blank for an aircraft that
/// does not provide one. The LEDs follow the user's bindings only.
/// </summary>
internal class Rmp32Renderer : FrontpanelRenderer
{
    private readonly Rmp32State _state = new();

    private readonly Rmp32Leds _leds = new();

    private RadioPanelState? _panel;

    public Rmp32Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void OnAttach(FlightDeckState model)
    {
        _panel = model.RadioPanel;
        foreach (var adapter in adapters)
        {
            if (adapter is Rmp32Adapter rmp) rmp.ControlActivated += OnControl;
        }
    }

    public override void OnDetach()
    {
        foreach (var adapter in adapters)
        {
            if (adapter is Rmp32Adapter rmp) rmp.ControlActivated -= OnControl;
        }
        _panel = null;
    }

    /// <summary>
    /// The upper ring tunes the standby in MHz and the lower ring in channel steps, as on a
    /// real RMP; the knob push copies the active frequency into the standby.
    /// </summary>
    private void OnControl(string controlId)
    {
        var panel = _panel;
        if (panel == null || !Enum.TryParse<Control>(controlId, out var control)) return;

        RadioPanelKey? key = control switch
        {
            Control.Swap => RadioPanelKey.Swap,
            Control.UpperRingRight => RadioPanelKey.CoarseUp,
            Control.UpperRingLeft => RadioPanelKey.CoarseDown,
            Control.LowerRingRight => RadioPanelKey.FineUp,
            Control.LowerRingLeft => RadioPanelKey.FineDown,
            Control.KnobPush => RadioPanelKey.Sync,
            Control.Adf => RadioPanelKey.ShowAdf,
            Control.Vhf1 or Control.Vhf2 or Control.Vhf3 => RadioPanelKey.ShowComm,
            _ => null,
        };

        if (key is RadioPanelKey k) panel.Press(k);
    }

    // Every display frame is a framebuffer write and a commit on the panel's 7-segment driver.
    // Sent ten times a second whether or not anything moved, and faster still while the
    // intercom selector sweeps the text, the panel stops redrawing. So a frame goes out only when
    // its text differs from the last one sent, and no more often than this.
    private static readonly TimeSpan MinFrameInterval = TimeSpan.FromMilliseconds(250);

    private (string Active, string Standby)? _lastSent;
    private long _lastSentAt;

    public override void Invalidate()
    {
        base.Invalidate();
        _lastSent = null;
    }

    public override void RenderDisplay(FlightDeckState model)
    {
        ApplyBrightness(model);

        var panel = model.RadioPanel;
        var active = panel?.Active ?? string.Empty;
        var standby = panel?.Standby ?? string.Empty;

        if (_lastSent == (active, standby)) return;

        // A change that comes too soon is not lost: the next tick still differs from what was
        // last sent, so the final value of a fast sweep always reaches the panel.
        var now = Environment.TickCount64;
        if (_lastSent != null && now - _lastSentAt < MinFrameInterval.TotalMilliseconds) return;

        _state.ActiveDisplay = active;
        _state.StandbyDisplay = standby;
        SendDisplay(_state);

        _lastSent = (active, standby);
        _lastSentAt = now;
    }

    public override void RenderLeds(FlightDeckState model)
    {
        var mask = BuildLeds(model, _leds, LedDeviceFamily.Rmp32);
        if (LedsChanged(mask)) SendLeds(_leds);
    }
}
