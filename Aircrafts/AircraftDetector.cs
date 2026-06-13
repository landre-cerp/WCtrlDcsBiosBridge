using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Interfaces;
using DCS_BIOS.Serialized;
using NLog;

namespace WWCduDcsBiosBridge.Aircrafts;

/// <summary>
/// Continuously monitors the <c>MetadataStart/_ACFT_NAME</c> string from
/// DCS-BIOS and exposes the currently loaded aircraft, signalling whenever it
/// changes (a new module, or exit → <c>null</c>).
///
/// The name comes from the string listener. Whether a module is actually loaded
/// comes from the raw data listener: the DCSFPCommon string buffer never clears
/// itself — once "complete" it keeps broadcasting the cached value even after
/// DCS zeroes the memory — so the first two raw bytes at the <c>_ACFT_NAME</c>
/// base address (0x0000 = no cockpit) are the reliable "is a module loaded" gate.
/// </summary>
internal sealed class AircraftDetector : IDCSBIOSStringListener, IDcsBiosDataListener, IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private readonly object _lock = new();

    private DCSBIOSOutput? _acftNameOutput;

    // Raw signals from DCS-BIOS.
    private bool _exited;        // latched true when the raw base bytes read 0 (cockpit unloaded)
    private string _name = "";   // last sanitized _ACFT_NAME string

    // Derived state surfaced to consumers.
    private string? _current;        // present ? valid name : null
    private string? _acknowledged;   // last value handed out via WaitForChangeAsync
    private bool _hasAcknowledged;   // distinguishes "acknowledged null" from "never acknowledged"
    private TaskCompletionSource<string?>? _changeTcs;

    private bool _loggedSanitize;
    private bool _disposed;

    /// <summary>
    /// Loads the MetadataStart controls, finds <c>_ACFT_NAME</c>, and starts listening.
    /// </summary>
    public void StartListening()
    {
        var metaControls = DCSBIOSControlLocator.GetMetaControls();
        var acftNameControl = metaControls.FirstOrDefault(c => c.Identifier == "_ACFT_NAME");

        if (acftNameControl == null)
        {
            Logger.Error("MetadataStart/_ACFT_NAME control not found in DCS-BIOS JSON files.");
            return;
        }

        _acftNameOutput = new DCSBIOSOutput();
        _acftNameOutput.Consume(acftNameControl, DCSBiosOutputType.StringType);

        BIOSEventHandler.AttachStringListener(this);
        BIOSEventHandler.AttachDataListener(this);
    }

    /// <summary>
    /// Completes with the current aircraft (the raw DCS-BIOS name, or <c>null</c>
    /// when no module is loaded) as soon as it differs from the value the caller
    /// last received. If the aircraft already changed since the last call it
    /// returns immediately, so callers never miss a transition.
    /// </summary>
    public Task<string?> WaitForChangeAsync(CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            if (!_hasAcknowledged || !string.Equals(_current, _acknowledged, StringComparison.Ordinal))
            {
                _acknowledged = _current;
                _hasAcknowledged = true;
                return Task.FromResult(_current);
            }

            _changeTcs = new TaskCompletionSource<string?>(TaskCreationOptions.RunContinuationsAsynchronously);
            return cancellationToken.CanBeCanceled
                ? _changeTcs.Task.WaitAsync(cancellationToken)
                : _changeTcs.Task;
        }
    }

    // --- String listener: gives the aircraft name ---

    public void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        if (_acftNameOutput == null || e.Address != _acftNameOutput.Address)
            return;

        var name = Sanitize(e.StringData);

        if (!_loggedSanitize && Logger.IsDebugEnabled && name.Length > 0
            && name != e.StringData?.TrimEnd('\0'))
        {
            _loggedSanitize = true;
            var hex = string.Join(" ", (e.StringData ?? "").Select(c => ((int)c).ToString("X2")));
            Logger.Debug($"_ACFT_NAME contained non-printable bytes, sanitized '{name}' from raw [{hex}]");
        }

        Update(name: name);
    }

    // --- Raw data listener: catches the exit transition ---
    // The string buffer never clears, so it keeps broadcasting the cached name
    // after the cockpit is unloaded. The raw base bytes going to 0 is the reliable
    // "exited" signal; they become non-zero again when the next module loads.
    // The latch defaults to "not exited" so an aircraft that was already loaded
    // when the bridge started is detected from the string alone.

    public void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        if (_acftNameOutput == null || e.Address != _acftNameOutput.Address)
            return;

        Update(exited: e.Data == 0);
    }

    /// <summary>
    /// Recomputes the current aircraft from the latest raw signals and, if it
    /// changed, wakes any pending <see cref="WaitForChangeAsync"/> caller.
    /// </summary>
    private void Update(bool? exited = null, string? name = null)
    {
        lock (_lock)
        {
            if (exited.HasValue) _exited = exited.Value;
            if (name != null) _name = name;

            var valid = !_exited && _name.Length > 0 && _name != "NONE";
            var next = valid ? _name : null;

            if (string.Equals(next, _current, StringComparison.Ordinal))
                return;

            _current = next;

            var waiter = _changeTcs;
            if (waiter != null)
            {
                _changeTcs = null;
                _acknowledged = _current;
                _hasAcknowledged = true;
                waiter.TrySetResult(_current);
            }
        }
    }

    /// <summary>
    /// DCS-BIOS occasionally pads <c>_ACFT_NAME</c> with non-printable or non-ASCII
    /// bytes (e.g. the F4U sends a trailing 0xA0). The CDU display protocol writes
    /// one fixed-size record per cell, so a single multi-byte char shifts the whole
    /// packet stream and scrambles the screen. Keep only printable ASCII.
    /// </summary>
    private static string Sanitize(string? raw)
    {
        if (string.IsNullOrEmpty(raw)) return string.Empty;
        var chars = raw.Where(c => c >= ' ' && c <= '~').ToArray();
        return new string(chars).Trim();
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        BIOSEventHandler.DetachStringListener(this);
        BIOSEventHandler.DetachDataListener(this);
    }
}
