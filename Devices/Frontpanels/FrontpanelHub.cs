using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Config;
using WWCduDcsBiosBridge.Devices.Frontpanels.Renderers;
using Timer = System.Timers.Timer;

namespace WWCduDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// Hosts one renderer per connected frontpanel family and periodically renders
/// the attached <see cref="FlightDeckState"/> to all of them. Aircraft listeners
/// only write semantic values into the model; they never see device types.
/// </summary>
public class FrontpanelHub : IDisposable
{
    private const double RenderIntervalMs = 100;

    private readonly List<IFrontpanelAdapter> _adapters;
    private readonly List<FrontpanelRenderer> _renderers = new();
    private readonly Timer _renderTimer;
    private readonly object _renderLock = new();

    private FlightDeckState? _model;
    private bool _disposed;

    /// <summary>
    /// Gets the collection of frontpanel adapters.
    /// </summary>
    public IReadOnlyList<IFrontpanelAdapter> Adapters => _adapters.AsReadOnly();

    /// <summary>
    /// Gets a value indicating whether any frontpanels are connected.
    /// </summary>
    public bool HasFrontpanels => _adapters.Count > 0;

    /// <summary>
    /// Gets the number of connected frontpanels.
    /// </summary>
    public int Count => _adapters.Count;

    public FrontpanelHub(IEnumerable<IFrontpanelAdapter> adapters, bool manageLighting)
    {
        _adapters = new List<IFrontpanelAdapter>(adapters ?? throw new ArgumentNullException(nameof(adapters)));

        CreateRenderer<FcuEfisAdapter>(a => new FcuEfisRenderer(a, manageLighting));
        CreateRenderer<Pap3Adapter>(a => new Pap3Renderer(a, manageLighting));
        CreateRenderer<Agp32Adapter>(a => new Agp32Renderer(a, manageLighting));
        CreateRenderer<Pdc3Adapter>(a => new Pdc3Renderer(a, manageLighting));

        var unhandled = _adapters.Where(a => a is not (FcuEfisAdapter or Pap3Adapter or Agp32Adapter or Pdc3Adapter)).ToList();
        foreach (var adapter in unhandled)
        {
            App.Logger.Warn($"No renderer for frontpanel adapter type: {adapter.GetType().Name} ({adapter.DisplayName})");
        }

        _renderTimer = new Timer(RenderIntervalMs);
        _renderTimer.Elapsed += (_, _) => RenderTick();
    }

    private void CreateRenderer<TAdapter>(Func<IReadOnlyList<IFrontpanelAdapter>, FrontpanelRenderer> create)
        where TAdapter : IFrontpanelAdapter
    {
        var familyAdapters = _adapters.Where(a => a is TAdapter).ToList();
        if (familyAdapters.Count == 0) return;

        _renderers.Add(create(familyAdapters));
        App.Logger.Info($"Frontpanel renderer created: {typeof(TAdapter).Name} x{familyAdapters.Count}");
    }

    /// <summary>
    /// Stops the render loop and detaches the model so a new one can be
    /// attached later (e.g. after an aircraft module change).
    /// </summary>
    internal void Detach()
    {
        _renderTimer.Stop();
        _model = null;
    }

    /// <summary>
    /// Attaches the semantic model to render and starts the render loop.
    /// </summary>
    internal void Attach(FlightDeckState model)
    {
        _model = model ?? throw new ArgumentNullException(nameof(model));
        if (_renderers.Count > 0)
        {
            _renderTimer.Start();
        }
    }

    private void RenderTick()
    {
        // Skip the tick instead of queueing behind a stalled USB write.
        if (!Monitor.TryEnter(_renderLock)) return;
        try
        {
            var model = _model;
            if (model == null) return;

            foreach (var renderer in _renderers)
            {
                try
                {
                    renderer.Render(model);
                }
                catch (Exception ex)
                {
                    App.Logger.Error(ex, $"Frontpanel renderer {renderer.GetType().Name} failed");
                }
            }
        }
        finally
        {
            Monitor.Exit(_renderLock);
        }
    }

    /// <summary>
    /// Stops the render loop and resets all connected frontpanel devices according
    /// to <paramref name="opts"/>. Call this before <see cref="Dispose"/> when a
    /// deliberate on-close reset is required.
    /// </summary>
    internal void ApplyCloseReset(CloseResetOptions opts)
    {
        _renderTimer.Stop();

        // Acquire the render lock so we don't race with an in-flight tick.
        lock (_renderLock)
        {
            foreach (var adapter in _adapters)
            {
                try
                {
                    // Reset() blanks screen + turns LEDs off at 100 % brightness for
                    // CommonWinWingPanel devices; for segment-only devices (AGP32)
                    // it only restores brightness, which is then overridden below.
                    adapter.Device.Reset();

                    var b = opts.BrightnessByte;
                    adapter.Device.SetBrightness(b, b, b);
                }
                catch (Exception ex)
                {
                    App.Logger.Warn(ex, $"Failed to reset frontpanel {adapter.DisplayName} on close");
                }
            }
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        _renderTimer.Stop();
        _renderTimer.Dispose();
        GC.SuppressFinalize(this);
    }
}
