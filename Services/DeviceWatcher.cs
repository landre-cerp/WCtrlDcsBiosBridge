using HidSharp;
using NLog;
using WwDevicesDotNet;
using Timer = System.Timers.Timer;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Watches the local USB HID bus for supported Winwing devices arriving or being
/// removed, and raises <see cref="DeviceArrived"/> / <see cref="DeviceRemoved"/> so the
/// app can hot-plug devices without a manual re-detect.
///
/// HidSharp's <see cref="DeviceList.Changed"/> fires several times per physical plug and
/// on a background thread, so changes are debounced and reconciled against the last-known
/// set keyed by <see cref="DeviceIdentifier"/>. Two physically identical panels share an
/// identifier and cannot be told apart (the connect path keys on VID/PID) — a pre-existing
/// limitation; distinct panel types (incl. MCDU Captain vs F/O) are fine.
///
/// Events are raised on a background (timer) thread; subscribers must marshal to their UI
/// thread as needed.
/// </summary>
public sealed class DeviceWatcher : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private const double DebounceMs = 300;

    private readonly object _lock = new();
    private readonly Timer _debounce;
    private List<DeviceIdentifier> _known = new();
    private bool _started;
    private bool _disposed;

    /// <summary>Raised when a supported device appears on the bus.</summary>
    public event Action<DeviceIdentifier>? DeviceArrived;

    /// <summary>Raised when a previously present supported device is removed.</summary>
    public event Action<DeviceIdentifier>? DeviceRemoved;

    public DeviceWatcher()
    {
        _debounce = new Timer(DebounceMs) { AutoReset = false };
        _debounce.Elapsed += (_, _) => Reconcile();
    }

    /// <summary>
    /// Snapshots the currently present devices as the known baseline (so already-connected
    /// devices are not reported as new) and begins watching for changes.
    /// </summary>
    public void Start()
    {
        lock (_lock)
        {
            if (_started || _disposed) return;
            _known = DeviceManager.FindLocalSupportedDevices().ToList();
            _started = true;
        }

        DeviceList.Local.Changed += OnHidDeviceListChanged;
        Logger.Info($"DeviceWatcher started; baseline {_known.Count} device(s).");
    }

private void OnHidDeviceListChanged(object? sender, DeviceListChangedEventArgs e)
{
    // Collapse the burst of Changed events a single plug emits into one reconcile.
    lock (_lock)
    {
        if (_disposed) return;
        _debounce.Stop();
        _debounce.Start();
    }
}

    private void Reconcile()
    {
        List<DeviceIdentifier> arrived;
        List<DeviceIdentifier> removed;

        lock (_lock)
        {
            if (_disposed) return;

            var current = DeviceManager.FindLocalSupportedDevices();
            arrived = current.Where(c => !_known.Any(k => k.Equals(c))).ToList();
            removed = _known.Where(k => !current.Any(c => c.Equals(k))).ToList();
            _known = current.ToList();
        }

        // Raise removals before arrivals so a replug (remove → add of the same unit)
        // tears down the stale device before the new one is connected.
foreach (var id in removed)
{
    Logger.Info($"Device removed: {id.Description}");
    try { DeviceRemoved?.Invoke(id); }
    catch (Exception ex) { Logger.Error(ex, $"DeviceRemoved handler failed for {id.Description}"); }
}

foreach (var id in arrived)
{
    Logger.Info($"Device arrived: {id.Description}");
    try { DeviceArrived?.Invoke(id); }
    catch (Exception ex) { Logger.Error(ex, $"DeviceArrived handler failed for {id.Description}"); }
}
    }

    public void Dispose()
    {
        lock (_lock)
        {
            if (_disposed) return;
            _disposed = true;
        }

        if (_started)
            DeviceList.Local.Changed -= OnHidDeviceListChanged;

        _debounce.Stop();
        _debounce.Dispose();
    }
}
