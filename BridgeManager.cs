using ClassLibraryCommon;
using DCS_BIOS;
using DCS_BIOS.ControlLocator;
using NLog;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Devices;
using WCtrlDcsBiosBridge.Devices.Cdu;
using WCtrlDcsBiosBridge.Devices.Frontpanels;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge;

/// <summary>
/// Manages the DCS-BIOS bridge lifecycle
/// </summary>
public class BridgeManager : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    /// <summary>
    /// True once an aircraft is detected and listeners are running. Drives the "running"
    /// UI state. It is <c>false</c> during the waiting phase even though the bridge loop
    /// is alive — use <see cref="IsLoopActive"/> for "is the bridge loop running at all".
    /// </summary>
    public bool IsStarted { get; private set; }

    /// <summary>
    /// True for the entire bridge lifetime — from the moment <see cref="StartAsync"/>
    /// enters its detection loop (the waiting phase included) until it stops. Hot-plug
    /// commands are only meaningful, and shutdown only needs to call <see cref="Stop"/>,
    /// while this is true.
    /// </summary>
    public bool IsLoopActive { get; private set; }

    internal List<CduDeviceContext>? Contexts { get; private set; }

    private DCSBIOS? dcsBios;
    private FrontpanelHub? frontpanelHub;
    private AircraftListener? headlessListener;
    private bool _manageLighting;
    private CloseResetOptions _closeReset = new(true, true);
    private bool _disposed = false;

    // Captured at start; used to build hot-added contexts / hubs and to re-source
    // the frontpanels when their model-source CDU is unplugged.
    private UserOptions _options = new();
    private bool _ch47SwitchWithSeat;

    // The live device set the bridge owns (mutated as devices are hot-plugged).
    private List<DeviceInfo>? _devices;

    // The currently running aircraft (null while waiting), and the listener whose
    // FlightDeck model currently drives the frontpanel hub. Both are only mutated on
    // the detection-loop thread.
    private AircraftDescriptor? _activeDescriptor;
    private AircraftListener? _modelSource;

    // Hot-plug commands enqueued from the UI thread and drained on the detection-loop
    // thread, so every CDU / listener / hub mutation happens on that one thread (this
    // also keeps CDU screen rebuilds off arbitrary threads). The signal wakes the loop
    // out of its blocking wait when a command is queued.
    private readonly System.Collections.Concurrent.ConcurrentQueue<Action> _deviceCommands = new();
    private readonly object _signalLock = new();
    private TaskCompletionSource<bool> _deviceSignalTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);

    // Coordinates CDU waiting-screen writes between the detection loop and the
    // DCS-BIOS version callback (which arrives on a background thread). Also guards
    // mutation of <see cref="Contexts"/> so the version callback never iterates a
    // list that a hot-plug add/remove is changing.
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
            _options = options;
            _manageLighting = !options.DisableLightingManagement;
            _closeReset = CloseResetOptions.From(options);
            _devices = devices.ToList();
            var cduDevices = _devices.Where(d => d.Cdu != null).ToList();

            // Zero or one CDU behave the same: no pilot/copilot prompt, and the
            // CH-47F uses seat-switch mode (follows the seat position at runtime).
            // Only multiple CDUs each pick a seat. A frontpanel-only setup (no CDU)
            // is driven by a headless listener using this same single-CDU logic.
            _ch47SwitchWithSeat = cduDevices.Count <= 1;

            // One context per CDU; frontpanel devices are driven by the hub below.
            Contexts = cduDevices.Select(d => new CduDeviceContext(d.Cdu!, options, _ch47SwitchWithSeat)).ToList();

            frontpanelHub = BuildFrontpanelHub(_devices, manageLighting: _manageLighting);
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
            using var detector = new DcsBiosAircraftDetector();
            detector.StartListening();

            // The version provider also lives for the whole lifecycle: the
            // DCS-BIOS exporter version is shown in the app title and on the CDU
            // waiting screen. Its callback arrives on a DCS-BIOS thread, so it is
            // serialized against the loop's waiting-screen writes.
            using var versionProvider = new DcsBiosVersionProvider();
            _dcsBiosVersion = versionProvider.CurrentVersion;
            versionProvider.VersionChanged += OnDcsBiosVersionChanged;
            versionProvider.StartListening();

            // The bridge loop is now live (waiting phase included); hot-plug commands
            // can be serviced and shutdown must call Stop().
            IsLoopActive = true;

            // Detection loop: wait for a SUPPORTED aircraft → start listeners →
            // wait for the aircraft to change (exit or switch) → reset → repeat.
            // Unsupported aircraft keep the bridge in the waiting state with the
            // detected name shown in red.
            while (!cancellationToken.IsCancellationRequested)
            {
                ShowWaitingScreens(null);
                DetectedAircraftChanged?.Invoke(null);

                // Stay in the waiting state until a supported aircraft is loaded.
                // A single detector waiter is kept across iterations (the detector
                // only supports one pending waiter); device hot-plug commands are
                // serviced in parallel without disturbing it.
                AircraftDescriptor? descriptor = null;
                var nameTask = detector.WaitForChangeAsync(cancellationToken);
                while (descriptor == null)
                {
                    var deviceTask = DeviceSignalTask(cancellationToken);
                    if (await Task.WhenAny(nameTask, deviceTask) == deviceTask)
                    {
                        await deviceTask;               // observe cancellation
                        DrainDeviceCommands();           // add/remove while waiting (cases B/E)
                        continue;                        // nameTask still pending
                    }

                    var name = await nameTask;
                    DetectedAircraftChanged?.Invoke(name);

                    if (name == null)
                    {
                        ShowWaitingScreens(null);
                        nameTask = detector.WaitForChangeAsync(cancellationToken);
                        continue;
                    }

                    descriptor = AircraftRegistry.FindByDcsBiosName(name);
                    Logger.Info($"DCS-BIOS: '{name}' -> {descriptor?.DisplayName ?? "unsupported"}");

                    if (descriptor == null)
                    {
                        ShowWaitingScreens(name);
                        nameTask = detector.WaitForChangeAsync(cancellationToken);
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

                // Recompute against the current CDU count: devices may have been
                // hot-plugged since the bridge started.
                var multipleCdus = Contexts.Count > 1;

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

                // Record the running aircraft first so the frontpanel source can be
                // derived and so hot-added devices can join it.
                _activeDescriptor = descriptor;
                RefreshFrontpanelSource(forceAttach: true);

                IsStarted = true;
                Logger.Info($"Bridge running with {ActiveDeviceCount} device(s)");

                // Block until the aircraft changes in DCS (module exit or switch),
                // reusing the waiter created above. While running, also service
                // device hot-plug commands without resetting the aircraft.
                while (true)
                {
                    var deviceTask = DeviceSignalTask(cancellationToken);
                    if (await Task.WhenAny(changeTask, deviceTask) == changeTask)
                    {
                        await changeTask;   // observe result / cancellation
                        break;
                    }

                    await deviceTask;        // observe cancellation
                    DrainDeviceCommands();   // add/remove while running (cases C/F)
                }
                Logger.Info("Aircraft changed, resetting bridge.");

                _activeDescriptor = null;
                StopListeners();
                foreach (var ctx in Contexts)
                    ctx.ResetForNewCycle();
            }
        }
        catch (OperationCanceledException)
        {
            // Normal shutdown via the cancellation token — not a failure, so don't
            // log it as an error. We must still release resources here: if the
            // cancellation arrived before IsStarted became true (e.g. exit from the
            // waiting screen), the owner's Dispose() skips Stop() (it is guarded by
            // IsStarted), so dcsBios / frontpanelHub / Contexts would otherwise leak.
            // dcsBios is non-null only when we have not been stopped yet, so this
            // also avoids stopping twice when the owner already did.
            if (dcsBios != null)
            {
                try { Stop(); }
                catch (Exception ex) { Logger.Error(ex, "Error stopping bridge after cancellation"); }
            }
            throw;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to start bridge");
            Stop(); // Clean up on failure
            throw;
        }
        finally
        {
            IsLoopActive = false;
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

            if (_manageLighting)
                frontpanelHub?.ApplyCloseReset(_closeReset);

            frontpanelHub?.Dispose();
            frontpanelHub = null;

            headlessListener?.Dispose();
            headlessListener = null;

            if (_manageLighting && Contexts != null)
            {
                foreach (var ctx in Contexts)
                {
                    try
                    {
                        var b = _closeReset.BrightnessPercent;
                        ctx?.Mcdu?.Cleanup(b, b, _closeReset.Markers ? 0 : 100);
                    }
                    catch (Exception ex) { Logger.Warn(ex, "Failed to reset CDU on stop"); }
                }
            }

            DisposeContexts();

            IsStarted = false;
            IsLoopActive = false;
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
        _modelSource = null;

        // IsStarted stays true on purpose: between two aircraft (module exit or
        // switch) the bridge is still live and the UI should keep showing
        // "running". It is only cleared by Stop().
    }

    /// <summary>
    /// Ensures the frontpanel hub is driven by a valid model source while an aircraft is
    /// active: a CDU listener if any CDU is running, otherwise a headless listener created
    /// for a frontpanel-only setup. No-op while waiting or when there are no frontpanels.
    /// Call after the hub or the CDU set changes.
    /// </summary>
    /// <param name="forceAttach">
    /// Re-attach even when the source is unchanged — needed after the hub itself was
    /// rebuilt (a new hub instance must be (re)attached to start its render loop).
    /// </param>
    private void RefreshFrontpanelSource(bool forceAttach = false)
    {
        if (frontpanelHub == null || !frontpanelHub.HasFrontpanels || _activeDescriptor == null)
            return;

        var source = Contexts?.FirstOrDefault(c => c.Listener != null)?.Listener;
        if (source == null)
        {
            if (headlessListener == null)
            {
                headlessListener = new AircraftListenerFactory().CreateListener(
                    new AircraftSelection(_activeDescriptor.ModuleId, true), null, _options, _ch47SwitchWithSeat);
                headlessListener.Start();
            }
            source = headlessListener;
        }

        var changed = !ReferenceEquals(_modelSource, source);
        _modelSource = source;

        if (changed || forceAttach)
            frontpanelHub.Attach(source.FlightDeck);

        // Once a CDU drives the panels, the headless listener (if any) is redundant.
        if (!ReferenceEquals(source, headlessListener) && headlessListener != null)
        {
            headlessListener.Dispose();
            headlessListener = null;
        }
    }

    // ---- Hot-plug: live device add / remove --------------------------------
    // Public Add/Remove enqueue a command and wake the detection loop; the command
    // is applied on the loop thread (DrainDeviceCommands), so every CDU/listener/hub
    // mutation — including CDU screen rebuilds — happens on that one thread.

    /// <summary>
    /// Queues a newly connected device to join the running bridge. Thread-safe.
    /// </summary>
    public void AddDevice(DeviceInfo device)
    {
        if (device == null || _disposed) return;
        _deviceCommands.Enqueue(() => ApplyAddDevice(device));
        SignalDeviceChange();
    }

    /// <summary>
    /// Queues removal of a device that was unplugged. Thread-safe.
    /// </summary>
    public void RemoveDevice(DeviceIdentifier deviceId)
    {
        if (deviceId == null || _disposed) return;
        _deviceCommands.Enqueue(() => ApplyRemoveDevice(deviceId));
        SignalDeviceChange();
    }

    /// <summary>Returns a task that completes when a device command is queued.</summary>
    private Task DeviceSignalTask(CancellationToken cancellationToken)
    {
        Task task;
        lock (_signalLock) { task = _deviceSignalTcs.Task; }
        return cancellationToken.CanBeCanceled ? task.WaitAsync(cancellationToken) : task;
    }

    private void SignalDeviceChange()
    {
        lock (_signalLock) { _deviceSignalTcs.TrySetResult(true); }
    }

    /// <summary>
    /// Applies every queued device command on the detection-loop thread, then re-arms
    /// the signal. If a command slipped in just after draining, the signal is re-raised
    /// so the next wait returns immediately and no command is lost.
    /// </summary>
    private void DrainDeviceCommands()
    {
        while (_deviceCommands.TryDequeue(out var apply))
        {
            try { apply(); }
            catch (Exception ex) { Logger.Error(ex, "Failed to apply a device hot-plug command"); }
        }

        lock (_signalLock)
        {
            if (_deviceSignalTcs.Task.IsCompleted)
                _deviceSignalTcs = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        }

        if (!_deviceCommands.IsEmpty) SignalDeviceChange();
    }

    private void ApplyAddDevice(DeviceInfo device)
    {
        if (_devices == null || Contexts == null) return;
        if (_devices.Any(d => d.DeviceId.Equals(device.DeviceId)))
        {
            Logger.Warn($"Ignoring duplicate hot-plug add for {device.DisplayName}");
            return;
        }
        _devices.Add(device);

        if (device.Cdu != null)
            AddCduContext(device.Cdu);
        else if (device.Frontpanel != null)
            RebuildFrontpanelHub();

        Logger.Info($"Hot-plug add applied: {device.DisplayName}; now {ActiveDeviceCount} device(s)");
    }

    private void ApplyRemoveDevice(DeviceIdentifier deviceId)
    {
        if (_devices == null || Contexts == null) return;

        var device = _devices.FirstOrDefault(d => d.DeviceId.Equals(deviceId));
        if (device == null) return;
        _devices.Remove(device);

        if (device.Cdu != null)
            RemoveCduContext(device.Cdu);
        else if (device.Frontpanel != null)
            RebuildFrontpanelHub();

        Logger.Info($"Hot-plug remove applied: {device.DisplayName}; now {ActiveDeviceCount} device(s)");
    }

    private void AddCduContext(ICdu cdu)
    {
        // CH-47 seat-switch mode applies only when this becomes the single CDU
        // (option (a): existing contexts keep their captured mode).
        var ctx = new CduDeviceContext(cdu, _options, ch47SwitchWithSeat: Contexts!.Count == 0);

        lock (_waitingScreenLock)
            Contexts.Add(ctx);

        if (_activeDescriptor == null)
        {
            // Waiting state: show the same waiting screen the other CDUs display.
            lock (_waitingScreenLock)
                ctx.ShowWaitingScreen(_waitingUnsupportedName, _dcsBiosVersion);
            return;
        }

        // Running state: join the active aircraft.
        if (_activeDescriptor.HasSeatSelection && Contexts.Count > 1)
        {
            ctx.ShowSeatSelectionScreen(_activeDescriptor);

            // The seat is chosen on a device-input thread; hop back onto the loop
            // thread to start the listener (keeps screen rebuilds on the loop thread).
            ctx.SelectionTask.ContinueWith(t =>
            {
                if (t.IsCompletedSuccessfully)
                {
                    _deviceCommands.Enqueue(() => StartHotAddedContext(ctx));
                    SignalDeviceChange();
                }
            }, TaskScheduler.Default);
        }
        else
        {
            ctx.SetAircraftSelection(new AircraftSelection(_activeDescriptor.ModuleId, true));
            StartHotAddedContext(ctx);
        }
    }

    private void StartHotAddedContext(CduDeviceContext ctx)
    {
        if (_activeDescriptor == null) return;   // aircraft changed before the seat pick
        if (!Contexts!.Contains(ctx)) return;    // context was removed (device unplugged) meanwhile

        ctx.StartBridge();
        // A CDU now exists; prefer its listener over any headless frontpanel source.
        RefreshFrontpanelSource();
    }

    private void RemoveCduContext(ICdu cdu)
    {
        var ctx = Contexts!.FirstOrDefault(c => ReferenceEquals(c.Mcdu, cdu));
        if (ctx == null) return;

        var wasSource = ctx.Listener != null && ReferenceEquals(_modelSource, ctx.Listener);

        lock (_waitingScreenLock)
            Contexts!.Remove(ctx);
        ctx.Dispose();   // disposes its listener; the USB device is closed by the owner

        if (wasSource)
        {
            _modelSource = null;       // force a re-pick (another CDU, or headless)
            RefreshFrontpanelSource();
        }
    }

    /// <summary>
    /// Rebuilds the frontpanel hub from the current device set (the hub captures its
    /// adapter/renderer lists at construction, so a changed frontpanel set needs a new
    /// hub). Disposing the old hub only stops its render loop — it never touches the USB
    /// devices — so the swap is safe and leaves CDUs and DCS-BIOS untouched.
    /// </summary>
    private void RebuildFrontpanelHub()
    {
        if (_devices == null) return;

        var oldHub = frontpanelHub;
        frontpanelHub = BuildFrontpanelHub(_devices, _manageLighting);
        oldHub?.Dispose();

        RefreshFrontpanelSource(forceAttach: true);
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
            // Stop whenever the loop is alive — including the waiting phase, where
            // IsStarted is still false but the DCS-BIOS foreground threads are running
            // and must be shut down or the process will not exit.
            if (IsLoopActive)
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
