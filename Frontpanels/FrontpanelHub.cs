using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Frontpanels.Renderers;
using Timer = System.Timers.Timer;

namespace WWCduDcsBiosBridge.Frontpanels;

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

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        _renderTimer.Stop();
        _renderTimer.Dispose();
        GC.SuppressFinalize(this);
    }
}
