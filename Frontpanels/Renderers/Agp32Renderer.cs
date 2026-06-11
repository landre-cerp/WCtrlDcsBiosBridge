using System.Collections.Generic;
using WwDevicesDotNet.Winctrl.Agp32;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.Frontpanels.Renderers;

/// <summary>
/// Renders the flight deck state to AGP32 clock/gear panels (A320 style).
/// </summary>
internal class Agp32Renderer : FrontpanelRenderer
{
    private readonly Agp32State _state = new();

    private readonly Dictionary<Agp32State.Agp32Led, bool> _desired = new();
    private readonly Dictionary<Agp32State.Agp32Led, bool> _sent = new();

    public Agp32Renderer(IReadOnlyList<IFrontpanelAdapter> adapters, bool manageLighting)
        : base(adapters, manageLighting)
    {
    }

    public override void Render(FlightDeckState model)
    {
        // Family policy: stay visible even when the cockpit console brightness
        // is at zero — the AGP32 gear lights are useless when dark.
        ApplyBrightness(255, 255, 255);

        if (model.GearLeftDown is bool left)
            _desired[Agp32State.Agp32Led.Gear1Down] = left;
        if (model.GearNoseDown is bool nose)
            _desired[Agp32State.Agp32Led.Gear2Down] = nose;
        if (model.GearRightDown is bool right)
            _desired[Agp32State.Agp32Led.Gear3Down] = right;
        if (model.GearWarning is bool warning)
        {
            // A single gear-in-transit warning maps to the A320 panel's red UNLK
            // triangles (lit while a gear disagrees with the lever) plus the
            // lever's red arrow.
            _desired[Agp32State.Agp32Led.Gear1Unlk] = warning;
            _desired[Agp32State.Agp32Led.Gear2Unlk] = warning;
            _desired[Agp32State.Agp32Led.Gear3Unlk] = warning;
            _desired[Agp32State.Agp32Led.GearDownRed] = warning;
        }

        // Only transmit LEDs whose state changed: each entry is one HID command,
        // and re-sending the full set every tick risks the device dropping the
        // tail of the burst.
        Agp32State.Agp32Leds? delta = null;
        foreach (var kvp in _desired)
        {
            if (_sent.TryGetValue(kvp.Key, out var sent) && sent == kvp.Value) continue;
            delta ??= new Agp32State.Agp32Leds();
            delta.Set(kvp.Key, kvp.Value);
            _sent[kvp.Key] = kvp.Value;
        }

        foreach (var adapter in adapters)
        {
            if (!adapter.IsConnected) continue;
            adapter.UpdateDisplay(_state);
            if (delta != null) adapter.UpdateLeds(delta);
        }
    }
}
