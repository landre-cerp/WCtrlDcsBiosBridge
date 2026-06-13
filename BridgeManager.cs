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

    // Coordinates CDU waiting-screen writes between the detection loop and the
    // DCS-BIOS version callback (which arrives on a background thread).
    private readonly object _waitingScreenLock = new();
    private bool _isWaiting;
    private string? _waitingUnsupportedName;
    private string? _dcsBiosVersion;

    /// <summary>
    /// Raised every time the detected aircraft changes (including on exit → null).
    /// The string is the raw DCS-BIOS name, or <c>null</c> when the user leaves a module.
    /// </summary>
    public event Action<string?>? DetectedAircraftChanged;

    /// <summary>
    /// Raised whenever the DCS-BIOS exporter version reported by DCS changes.
    /// The string is the version (e.g. "0.11.4"), or <c>null</c> when not yet known.
    /// </summary>
    public event Action<string?>? DcsBiosVersionChanged;

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

            // The version provider also lives for the whole lifecycle: the
            // DCS-BIOS exporter version is shown in the app title and on the CDU
            // waiting screen. Its callback arrives on a DCS-BIOS thread, so it is
            // serialized against the loop's waiting-screen writes.
            using var versionProvider = new DcsBiosVersionProvider();
            _dcsBiosVersion = versionProvider.CurrentVersion;
            versionProvider.VersionChanged += OnDcsBiosVersionChanged;
            versionProvider.StartListening();

            // Detection loop: wait for a SUPPORTED aircraft → start listeners →
            // wait for the aircraft to change (exit or switch) → reset → repeat.
            // Unsupported aircraft keep the bridge in the waiting state with the
            // detected name shown in red.
            while (!cancellationToken.IsCancellationRequested)
            {
                ShowWaitingScreens(null);
                DetectedAircraftChanged?.Invoke(null);

                // Stay in the waiting state until a supported aircraft is loaded.
                AircraftDescriptor? descriptor = null;
                while (descriptor == null)
                {
                    var name = await detector.WaitForChangeAsync(cancellationToken);
                    DetectedAircraftChanged?.Invoke(name);

                    if (name == null)
                    {
                        ShowWaitingScreens(null);
                        continue;
                    }

                    descriptor = AircraftRegistry.FindByDcsBiosName(name);
                    Logger.Info($"DCS-BIOS: '{name}' -> {descriptor?.DisplayName ?? "unsupported"}");

                    if (descriptor == null)
                    {
                        ShowWaitingScreens(name);
                    }
                }

                // A supported aircraft is loaded: leave the waiting state so a
                // late version update no longer redraws the waiting screen.
                EndWaitingState();

                // One change waiter for this whole cycle: reused for the seat-
                // selection race below AND the running-phase wait further down.
                // The detector only supports a single pending waiter, so creating
                // it once here (instead of a second WaitForChangeAsync later)
                // avoids orphaning a waiter and corrupting its acknowledgment state.
                var changeTask = detector.WaitForChangeAsync(cancellationToken);

                if (descriptor.HasSeatSelection && multipleCdus)
                {
                    foreach (var ctx in Contexts)
                        ctx.ShowSeatSelectionScreen(descriptor);

                    // Race the seat choice against an aircraft change. If the user
                    // leaves the module before picking a seat, abandon the selection
                    // and return to the detection state instead of blocking forever.
                    var selectionTask = Task.WhenAny(
                        Contexts.Select(c => c.SelectionTask.WaitAsync(cancellationToken)));

                    var winner = await Task.WhenAny(selectionTask, changeTask);
                    if (winner == changeTask)
                    {
                        Logger.Info("Aircraft changed during seat selection, resetting bridge.");
                        foreach (var ctx in Contexts)
                            ctx.ResetForNewCycle();
                        continue;
                    }

                    var firstDone = await selectionTask;
                    var firstChoice = await firstDone;

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

                // Block here until the aircraft changes in DCS (module exit or
                // switch), reusing the waiter created above.
                await changeTask;
                Logger.Info("Aircraft changed, resetting bridge.");

                StopListeners();
                foreach (var ctx in Contexts)
                    ctx.ResetForNewCycle();
            }
        }
        catch (OperationCanceledException)
        {
            // Normal shutdown via the cancellation token: the owner (MainWindow)
            // already cancelled and calls Stop()/Dispose() itself, so this is not a
            // failure — don't log an error or stop twice, just unwind.
            throw;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to start bridge");
            Stop(); // Clean up on failure
            throw;
        }
    }

    /// <summary>
    /// Renders the waiting screen on every CDU with the current DCS-BIOS version,
    /// and records the waiting state so a background version update can redraw it.
    /// Serialized via <see cref="_waitingScreenLock"/> against the version callback.
    /// </summary>
    private void ShowWaitingScreens(string? unsupportedName)
    {
        if (Contexts == null) return;

        lock (_waitingScreenLock)
        {
            _isWaiting = true;
            _waitingUnsupportedName = unsupportedName;
            foreach (var ctx in Contexts)
                ctx.ShowWaitingScreen(unsupportedName, _dcsBiosVersion);
        }
    }

    /// <summary>
    /// Leaves the waiting state so a late DCS-BIOS version update no longer
    /// redraws the (now superseded) waiting screen.
    /// </summary>
    private void EndWaitingState()
    {
        lock (_waitingScreenLock)
        {
            _isWaiting = false;
            _waitingUnsupportedName = null;
        }
    }

    /// <summary>
    /// Caches the latest DCS-BIOS version, forwards it to the UI, and—if a CDU is
    /// still showing the waiting screen—redraws it so the version appears live.
    /// </summary>
    private void OnDcsBiosVersionChanged(string? version)
    {
        lock (_waitingScreenLock)
        {
            _dcsBiosVersion = version;

            if (_isWaiting && Contexts != null)
            {
                foreach (var ctx in Contexts)
                    ctx.ShowWaitingScreen(_waitingUnsupportedName, _dcsBiosVersion);
            }
        }

        DcsBiosVersionChanged?.Invoke(version);
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

        //IsStarted = false;
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
