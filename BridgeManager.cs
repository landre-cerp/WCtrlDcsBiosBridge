using ClassLibraryCommon;
using DCS_BIOS;
using DCS_BIOS.ControlLocator;
using NLog;
using WWCduDcsBiosBridge.Config;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge;

/// <summary>
/// Manages the DCS-BIOS bridge lifecycle
/// </summary>
public class BridgeManager : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    public bool IsStarted { get; private set; }
    internal List<DeviceContext>? Contexts { get; private set; }

    private DCSBIOS? dcsBios;
    private bool _disposed = false;
    private TaskCompletionSource<AircraftSelection>? _globalAircraftSelectionTcs;
    private AircraftSelection? _pendingGlobalAircraftSelection;

    /// <summary>
    /// Sets the global aircraft selection (used when no CDU is present)
    /// </summary>
    public void SetGlobalAircraftSelection(AircraftSelection selection)
    {
        _pendingGlobalAircraftSelection = selection;
        _globalAircraftSelectionTcs?.TrySetResult(selection);
    }

    /// <summary>
    /// Starts the bridge with the specified devices and configuration
    /// </summary>
    public async Task StartAsync(List<DeviceInfo> devices, UserOptions userOptions, DcsBiosConfig config, CancellationToken cancellationToken = default)
    {
        if (IsStarted)
            throw new InvalidOperationException("Bridge is already started");

        if (devices == null || !devices.Any())
            throw new ArgumentException("No devices provided");

        if (config == null)
            throw new ArgumentNullException(nameof(config));

        try
        {
            // Create device contexts for all devices
            Contexts = new List<DeviceContext>();

            // With a single CDU the CH-47F uses seat-switch mode: one menu entry,
            // the CDU display follows the seat position at runtime.
            var singleCdu = devices.Count(d => d.Cdu != null) == 1;
            var options = userOptions ?? new UserOptions();

            foreach (var deviceInfo in devices)
            {
                DeviceContext ctx;
                if (deviceInfo.Cdu != null)
                {
                    ctx = new DeviceContext(deviceInfo.Cdu, options, singleCdu);
                }
                else if (deviceInfo.Frontpanel != null)
                {
                    ctx = new DeviceContext(deviceInfo.Frontpanel, options);
                }
                else
                {
                    Logger.Warn("Skipping device with no CDU or Frontpanel interface");
                    continue;
                }
                Contexts.Add(ctx);
            }

            if (!Contexts.Any())
            {
                throw new InvalidOperationException("No valid devices found.");
            }

            var cduCount = Contexts.Count(c => c.IsCduDevice);
            var frontpanelCount = Contexts.Count(c => c.IsFrontpanelDevice);

            Logger.Info($"Created contexts for {cduCount} CDU device(s) and {frontpanelCount} Frontpanel device(s)");

            // Show startup screens only on CDU devices
            foreach (var ctx in Contexts.Where(c => c.IsCduDevice))
                ctx.ShowStartupScreen();

            // Wait for aircraft selection
            // If multiple CDUs are present, each must make its own selection
            // (important for CH47F with pilot and copilot CDUs)
            var cduContexts = Contexts.Where(c => c.IsCduDevice).ToList();

            if (cduContexts.Any())
            {
                Logger.Info($"Waiting for aircraft selection on {cduContexts.Count} CDU device(s)...");
                await Task.WhenAll(cduContexts.Select(c => c.SelectionTask.WaitAsync(cancellationToken)));

                for (int i = 0; i < cduContexts.Count; i++)
                {
                    var selection = cduContexts[i].SelectedAircraft;
                    Logger.Info($"  CDU {i + 1}: Aircraft={selection!.AircraftId}, IsPilot={selection.IsPilot}");
                }

                // Frontpanel-only devices follow the first CDU's selection
                var firstCduSelection = cduContexts[0].SelectedAircraft;
                foreach (var ctx in Contexts.Where(c => c.IsFrontpanelDevice))
                {
                    ctx.SetAircraftSelection(firstCduSelection!);
                }
            }
            else
            {
                // No CDU devices - wait for global UI selection
                Logger.Info("No CDU devices found. Waiting for global aircraft selection from UI...");
                _globalAircraftSelectionTcs = new TaskCompletionSource<AircraftSelection>(TaskCreationOptions.RunContinuationsAsynchronously);
                if (_pendingGlobalAircraftSelection != null)
                {
                    _globalAircraftSelectionTcs.TrySetResult(_pendingGlobalAircraftSelection);
                }
                var selectedAircraft = await _globalAircraftSelectionTcs.Task.WaitAsync(cancellationToken);
                Logger.Info($"Global aircraft selection received from UI: {selectedAircraft.AircraftId}, IsPilot: {selectedAircraft.IsPilot}");

                // Propagate to all frontpanel devices
                foreach (var ctx in Contexts.Where(c => c.IsFrontpanelDevice))
                {
                    ctx.SetAircraftSelection(selectedAircraft);
                }
            }

            // Global DCS-BIOS metadata initialization, once for all contexts
            DCSAircraft.Init();
            DCSAircraft.FillModulesListFromDcsBios(config.DcsBiosJsonLocation, true);
            DCSBIOSControlLocator.JSONDirectory = config.DcsBiosJsonLocation;

            // Initialize DCS-BIOS
            InitializeDcsBios(config);

            // Build the frontpanel hub from all frontpanel devices
            var frontpanelHub = BuildFrontpanelHub(Contexts);
            Logger.Info($"Frontpanel hub created with {frontpanelHub.Count} device(s)");

            // Start device bridges - pass hub to all contexts
            // We ensure only ONE listener drives the frontpanel hub to avoid race conditions and double-updates.
            // Priority: First CDU context, otherwise First Frontpanel context.
            // Since all contexts use the same shared FrontpanelHub instance which contains ALL connected frontpanels,
            // we only need one listener to feed it.
            var masterContext = Contexts.FirstOrDefault(c => c.IsCduDevice) ?? Contexts.FirstOrDefault(c => c.IsFrontpanelDevice);
            var emptyHub = FrontpanelHub.CreateEmpty();

            foreach (var ctx in Contexts)
            {
                if (ctx == masterContext)
                {
                    // This listener will drive all frontpanels in the hub
                    Logger.Info($"Starting bridge for master context (driving frontpanels): {ctx.SelectedAircraft?.AircraftId}");
                    ctx.StartBridge(frontpanelHub);
                }
                else
                {
                    // This listener will allow CDU to work but won't touch frontpanels
                    // For pure frontpanel contexts that are not master, this listener runs but writes to nowhere
                    ctx.StartBridge(emptyHub);
                }
            }

            IsStarted = true;
            Logger.Info($"Bridge started successfully with {Contexts.Count} device(s) ({cduCount} CDU, {frontpanelCount} Frontpanel)");
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to start bridge");
            Stop(); // Clean up on failure
            throw;
        }
    }

    /// <summary>
    /// Gets the number of active contexts
    /// </summary>
    public int ContextCount => Contexts?.Count ?? 0;

    /// <summary>
    /// Stops the bridge and cleans up resources
    /// </summary>
    public void Stop()
    {
        try
        {
            dcsBios?.Shutdown();
            dcsBios = null;

            DisposeContexts();

            IsStarted = false;
            Logger.Info("Bridge stopped successfully");
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Error occurred while stopping bridge");
            throw;
        }
    }

    private void InitializeDcsBios(DcsBiosConfig config)
    {
        dcsBios = new DCSBIOS(config.ReceiveFromIpUdp, config.SendToIpUdp,
                             config.ReceivePortUdp, config.SendPortUdp,
                             DcsBiosNotificationMode.Parse);

        if (!dcsBios.HasLastException())
        {
            if (!dcsBios.IsRunning)
            {
                dcsBios.Startup();
            }
            Logger.Info("DCS-BIOS started successfully.");
        }
        else
        {
            var exception = dcsBios.GetLastException();
            Logger.Error(exception);
            throw exception;
        }
    }

    private FrontpanelHub BuildFrontpanelHub(List<DeviceContext> contexts)
    {
        var adapters = new List<IFrontpanelAdapter>();

        foreach (var ctx in contexts.Where(c => c.IsFrontpanelDevice && c.Frontpanel != null))
        {
            var frontpanel = ctx.Frontpanel!;
            var adapter = FrontpanelAdapterFactory.CreateAdapter(frontpanel, frontpanel.DeviceId.ToString());
            if (adapter != null)
            {
                adapters.Add(adapter);
                Logger.Info($"Added frontpanel adapter: {adapter.DisplayName}");
            }
            else
            {
                Logger.Warn($"Unknown frontpanel type, skipping: {frontpanel.GetType().Name}");
            }
        }

        return new FrontpanelHub(adapters);
    }

    private void DisposeContexts()
    {
        if (Contexts != null)
        {
            foreach (var ctx in Contexts)
                ctx?.Dispose();
            Contexts = null;
        }
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
        if (_disposed)
            return;

        if (disposing)
        {
            if (IsStarted)
            {
                try
                {
                    Stop();
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, "Error stopping bridge during dispose");
                }
            }
        }

        _disposed = true;
    }
}
