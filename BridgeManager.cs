using ClassLibraryCommon;
using DCS_BIOS;
using DCS_BIOS.ControlLocator;
using NLog;
using WWCduDcsBiosBridge.Config;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Devices.Frontpanels;

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
    private FrontpanelHub? frontpanelHub;
    private AircraftListener? headlessListener;
    private bool _disposed = false;
    private TaskCompletionSource<AircraftSelection>? _globalAircraftSelectionTcs;
    private AircraftSelection? _pendingGlobalAircraftSelection;

    /// <summary>
    /// Gets the number of devices the bridge is driving (CDUs + frontpanels)
    /// </summary>
    public int ActiveDeviceCount => (Contexts?.Count ?? 0) + (frontpanelHub?.Count ?? 0);

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
            var options = userOptions ?? new UserOptions();
            var cduDevices = devices.Where(d => d.Cdu != null).ToList();

            // With a single CDU the CH-47F uses seat-switch mode: one menu entry,
            // the CDU display follows the seat position at runtime.
            var singleCdu = cduDevices.Count == 1;
            var ch47SwitchWithSeat = singleCdu;

            // One context per CDU; frontpanel devices are driven by the hub below.
            Contexts = cduDevices.Select(d => new DeviceContext(d.Cdu!, options, ch47SwitchWithSeat)).ToList();

            frontpanelHub = BuildFrontpanelHub(devices, manageLighting: !options.DisableLightingManagement);
            Logger.Info($"Created {Contexts.Count} CDU context(s); frontpanel hub has {frontpanelHub.Count} device(s)");

            if (Contexts.Count == 0 && !frontpanelHub.HasFrontpanels)
            {
                throw new InvalidOperationException("No valid devices found.");
            }

            // Wait for aircraft selection
            AircraftSelection firstSelection;
            if (Contexts.Any())
            {
                foreach (var ctx in Contexts)
                    ctx.ShowStartupScreen();

                // Each CDU makes its own selection
                // (important for CH47F with pilot and copilot CDUs)
                Logger.Info($"Waiting for aircraft selection on {Contexts.Count} CDU device(s)...");
                await Task.WhenAll(Contexts.Select(c => c.SelectionTask.WaitAsync(cancellationToken)));

                for (int i = 0; i < Contexts.Count; i++)
                {
                    var selection = Contexts[i].SelectedAircraft;
                    Logger.Info($"  CDU {i + 1}: Aircraft={selection!.AircraftId}, IsPilot={selection.IsPilot}");
                }

                firstSelection = Contexts[0].SelectedAircraft!;
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
                firstSelection = await _globalAircraftSelectionTcs.Task.WaitAsync(cancellationToken);
                Logger.Info($"Global aircraft selection received from UI: {firstSelection.AircraftId}, IsPilot: {firstSelection.IsPilot}");
            }

            // Global DCS-BIOS metadata initialization, once for all listeners
            DCSAircraft.Init();
            DCSAircraft.FillModulesListFromDcsBios(config.DcsBiosJsonLocation, true);
            DCSBIOSControlLocator.JSONDirectory = config.DcsBiosJsonLocation;

            // Initialize DCS-BIOS
            InitializeDcsBios(config);

            foreach (var ctx in Contexts)
            {
                ctx.StartBridge();
            }

            // Frontpanels are rendered from one listener's FlightDeck model:
            // the first CDU listener, or a headless listener when no CDU is present.
            if (frontpanelHub.HasFrontpanels)
            {
                var modelSource = Contexts.FirstOrDefault(c => c.Listener != null)?.Listener;
                if (modelSource == null)
                {
                    Logger.Info($"Starting headless listener for frontpanels: {firstSelection.AircraftId}");
                    headlessListener = new AircraftListenerFactory().CreateListener(firstSelection, null, options, ch47SwitchWithSeat);
                    headlessListener.Start();
                    modelSource = headlessListener;
                }

                frontpanelHub.Attach(modelSource.FlightDeck);
            }

            IsStarted = true;
            Logger.Info($"Bridge started successfully with {ActiveDeviceCount} device(s) ({Contexts.Count} CDU, {frontpanelHub.Count} Frontpanel)");
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to start bridge");
            Stop(); // Clean up on failure
            throw;
        }
    }

    /// <summary>
    /// Stops the bridge and cleans up resources
    /// </summary>
    public void Stop()
    {
        try
        {
            dcsBios?.Shutdown();
            dcsBios = null;

            frontpanelHub?.Dispose();
            frontpanelHub = null;

            headlessListener?.Dispose();
            headlessListener = null;

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

    private static FrontpanelHub BuildFrontpanelHub(List<DeviceInfo> devices, bool manageLighting)
    {
        var adapters = new List<IFrontpanelAdapter>();

        foreach (var frontpanel in devices.Where(d => d.Frontpanel != null).Select(d => d.Frontpanel!))
        {
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

        return new FrontpanelHub(adapters, manageLighting);
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
