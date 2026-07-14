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
    /// True while the bridge loop is active and DCS-BIOS is connected, including the
    /// phase where the bridge is waiting for a supported aircraft to be loaded.
    /// Becomes <c>false</c> only after <see cref="Stop"/> is called.
    /// Use <see cref="IsLoopActive"/> to distinguish "loop started at all" from
    /// "aircraft listeners are currently running" (see <see cref="Contexts"/>).
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

    private UserOptions _options = new();
    private bool _ch47SwitchWithSeat;

    private List<DeviceInfo>? _devices;

    private AircraftDescriptor? _activeDescriptor;
    private AircraftListener? _modelSource;

    
    private readonly System.Collections.Concurrent.ConcurrentQueue<Action> _deviceCommands = new();
    private readonly object _signalLock = new();
    private TaskCompletionSource<bool> _deviceSignalTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);

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
            _options = userOptions ?? new UserOptions();
            _manageLighting = !_options.DisableLightingManagement;
            _closeReset = CloseResetOptions.From(_options);
            _devices = devices.ToList();
            var cduDevices = _devices.Where(d => d.Cdu != null).ToList();

            _ch47SwitchWithSeat = cduDevices.Count <= 1;

            Contexts = cduDevices.Select(d => new CduDeviceContext(d.Cdu!, _options, _ch47SwitchWithSeat)).ToList();

            frontpanelHub = BuildFrontpanelHub(_devices, manageLighting: _manageLighting);
            Logger.Info($"Created {Contexts.Count} CDU context(s); frontpanel hub has {frontpanelHub.Count} device(s)");

            if (Contexts.Count == 0 && !frontpanelHub.HasFrontpanels)
            {
                throw new InvalidOperationException("No valid devices found.");
            }

            DCSAircraft.Init();
            DCSAircraft.FillModulesListFromDcsBios(config.DcsBiosJsonLocation, true);
            DCSBIOSControlLocator.JSONDirectory = config.DcsBiosJsonLocation;

            InitializeDcsBios(config);

            using var detector = new DcsBiosAircraftDetector();
            detector.StartListening();

            using var versionProvider = new DcsBiosVersionProvider();
            _dcsBiosVersion = versionProvider.CurrentVersion;
            versionProvider.VersionChanged += OnDcsBiosVersionChanged;
            versionProvider.StartListening();

            IsLoopActive = true;

            while (!cancellationToken.IsCancellationRequested)
            {
                ShowWaitingScreens(null);
                DetectedAircraftChanged?.Invoke(null);

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

                EndWaitingState();
                var changeTask = detector.WaitForChangeAsync(cancellationToken);

                var multipleCdus = Contexts.Count > 1;

                if (descriptor.HasSeatSelection && multipleCdus)
                {
                    foreach (var ctx in Contexts)
                        ctx.ShowSeatSelectionScreen(descriptor);

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

                _activeDescriptor = descriptor;
                RefreshFrontpanelSource(forceAttach: true);

                IsStarted = true;
                Logger.Info($"Bridge running with {ActiveDeviceCount} device(s)");

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
