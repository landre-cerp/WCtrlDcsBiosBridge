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

    /// <summary>
    /// Raised every time the detected aircraft changes (including on exit → null).
    /// The string is the raw DCS-BIOS name, or <c>null</c> when the user leaves a module.
    /// </summary>
    public event Action<string?>? DetectedAircraftChanged;

    /// <summary>
    /// Gets the number of devices the bridge is driving (CDUs + frontpanels)
    /// </summary>
    public int ActiveDeviceCount => (Contexts?.Count ?? 0) + (frontpanelHub?.Count ?? 0);

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

            // Zero or one CDU behave the same: no pilot/copilot prompt, and the
            // CH-47F uses seat-switch mode (follows the seat position at runtime).
            // Only multiple CDUs each pick a seat. A frontpanel-only setup (no CDU)
            // is driven by a headless listener using this same single-CDU logic.
            var multipleCdus = cduDevices.Count > 1;
            var ch47SwitchWithSeat = !multipleCdus;

            // One context per CDU; frontpanel devices are driven by the hub below.
            Contexts = cduDevices.Select(d => new DeviceContext(d.Cdu!, options, ch47SwitchWithSeat)).ToList();

            frontpanelHub = BuildFrontpanelHub(devices, manageLighting: !options.DisableLightingManagement);
            Logger.Info($"Created {Contexts.Count} CDU context(s); frontpanel hub has {frontpanelHub.Count} device(s)");

            if (Contexts.Count == 0 && !frontpanelHub.HasFrontpanels)
            {
                throw new InvalidOperationException("No valid devices found.");
            }

            // Global DCS-BIOS metadata initialization — needed before aircraft
            // detection AND before any listener can create its outputs.
            DCSAircraft.Init();
            DCSAircraft.FillModulesListFromDcsBios(config.DcsBiosJsonLocation, true);
            DCSBIOSControlLocator.JSONDirectory = config.DcsBiosJsonLocation;

            // Start the UDP connection so we can receive metadata.
            InitializeDcsBios(config);

            // The detector lives for the entire bridge lifecycle so it can
            // signal both aircraft detection and module exit.
            using var detector = new AircraftDetector();
            detector.StartListening();

            // Detection loop: wait for a SUPPORTED aircraft → start listeners →
            // wait for the aircraft to change (exit or switch) → reset → repeat.
            // Unsupported aircraft keep the bridge in the waiting state with the
            // detected name shown in red.
            while (!cancellationToken.IsCancellationRequested)
            {
                foreach (var ctx in Contexts)
                    ctx.ShowWaitingScreen();
                DetectedAircraftChanged?.Invoke(null);

                // Stay in the waiting state until a supported aircraft is loaded.
                AircraftDescriptor? descriptor = null;
                while (descriptor == null)
                {
                    var name = await detector.WaitForChangeAsync(cancellationToken);
                    DetectedAircraftChanged?.Invoke(name);

                    if (name == null)
                    {
                        foreach (var ctx in Contexts)
                            ctx.ShowWaitingScreen();
                        continue;
                    }

                    descriptor = AircraftRegistry.FindByDcsBiosName(name);
                    Logger.Info($"DCS-BIOS: '{name}' -> {descriptor?.DisplayName ?? "unsupported"}");

                    if (descriptor == null)
                    {
                        foreach (var ctx in Contexts)
                            ctx.ShowWaitingScreen(name);
                    }
                }

                if (descriptor.HasSeatSelection && multipleCdus)
                {
                    foreach (var ctx in Contexts)
                        ctx.ShowSeatSelectionScreen(descriptor);

                    using var seatWaitCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
                    var firstSelectionTask = Task.WhenAny(
                        Contexts.Select(c => c.SelectionTask.WaitAsync(cancellationToken)));
                    var aircraftChangedTask = detector.WaitForChangeAsync(seatWaitCts.Token);

                    var completedTask = await Task.WhenAny(firstSelectionTask, aircraftChangedTask);
                    if (completedTask == aircraftChangedTask)
                    {
                        var name = await aircraftChangedTask;
                        DetectedAircraftChanged?.Invoke(name);
                        Logger.Info("Aircraft changed before seat selection completed, restarting detection.");
                        foreach (var ctx in Contexts)
                            ctx.ResetForNewCycle();
                        continue;
                    }

                    seatWaitCts.Cancel();
                    var firstChoice = await await firstSelectionTask;

                    var oppositeSeat = !firstChoice.IsPilot;
                    foreach (var ctx in Contexts.Where(c => !c.IsSelectedAircraft))
                        ctx.SetAircraftSelection(new AircraftSelection(descriptor.ModuleId, oppositeSeat));
                }
                else
                {
                    foreach (var ctx in Contexts)
                        ctx.SetAircraftSelection(new AircraftSelection(descriptor.ModuleId, true));
                }

                foreach (var ctx in Contexts)
                    ctx.StartBridge();

                AttachFrontpanels(Contexts, new AircraftSelection(descriptor.ModuleId, true), options, ch47SwitchWithSeat);

                IsStarted = true;
                Logger.Info($"Bridge running with {ActiveDeviceCount} device(s)");

                // Block here until the aircraft changes in DCS (module exit or switch).
                await detector.WaitForChangeAsync(cancellationToken);
                Logger.Info("Aircraft changed, resetting bridge.");

                StopListeners();
                foreach (var ctx in Contexts)
                    ctx.ResetForNewCycle();
            }
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

    /// <summary>
    /// Stops aircraft listeners and detaches frontpanels, but keeps the
    /// DCS-BIOS connection and device contexts alive for the next cycle.
    /// </summary>
    private void StopListeners()
    {
        frontpanelHub?.Detach();

        headlessListener?.Dispose();
        headlessListener = null;

        IsStarted = false;
    }

    /// <summary>
    /// Drives the frontpanels from a CDU listener's FlightDeck model when one is
    /// present, otherwise from a headless listener created for <paramref name="fallback"/>
    /// (e.g. a frontpanel-only setup with no CDU).
    /// </summary>
    private void AttachFrontpanels(List<DeviceContext> contexts, AircraftSelection fallback, UserOptions options, bool ch47SwitchWithSeat)
    {
        if (frontpanelHub == null || !frontpanelHub.HasFrontpanels) return;

        var modelSource = contexts.FirstOrDefault(c => c.Listener != null)?.Listener;
        if (modelSource == null)
        {
            var selection = contexts.FirstOrDefault()?.SelectedAircraft ?? fallback;
            headlessListener = new AircraftListenerFactory().CreateListener(selection, null, options, ch47SwitchWithSeat);
            headlessListener.Start();
            modelSource = headlessListener;
        }

        if (modelSource != null)
            frontpanelHub.Attach(modelSource.FlightDeck);
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
