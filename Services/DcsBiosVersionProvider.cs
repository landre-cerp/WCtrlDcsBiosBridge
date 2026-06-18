using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Interfaces;
using DCS_BIOS.Serialized;
using NLog;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Listens to the DCS-BIOS CommonData <c>VERSION</c> string and exposes the
/// exporter version reported by DCS (e.g. "0.11.4"), signalling whenever it
/// changes. The value lives in CommonData (a meta-module) so it is available
/// while the bridge is still waiting for an aircraft to load.
/// </summary>
internal sealed class DcsBiosVersionProvider : IDCSBIOSStringListener, IDisposable
{
    private const string VersionControlId = "VERSION";

    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private readonly object _lock = new();

    private DCSBIOSOutput? _versionOutput;
    private string? _version;
    private bool _disposed;

    /// <summary>
    /// Raised whenever the reported DCS-BIOS version changes. The argument is the
    /// sanitized version string, or <c>null</c> when no value has been received.
    /// </summary>
    public event Action<string?>? VersionChanged;

    /// <summary>
    /// The last DCS-BIOS exporter version received, or <c>null</c> if none yet.
    /// </summary>
    public string? CurrentVersion
    {
        get
        {
            lock (_lock)
            {
                return _version;
            }
        }
    }

    /// <summary>
    /// Resolves the <c>VERSION</c> meta-control and starts listening. Safe to call
    /// once per provider; logs and no-ops if the control is not present.
    /// </summary>
    public void StartListening()
    {
        var metaControls = DCSBIOSControlLocator.GetMetaControls();
        var versionControl = metaControls.FirstOrDefault(c => c.Identifier == VersionControlId);

        if (versionControl == null)
        {
            Logger.Warn("CommonData/VERSION control not found in DCS-BIOS JSON files; DCS-BIOS version will be unavailable.");
            return;
        }

        _versionOutput = new DCSBIOSOutput();
        _versionOutput.Consume(versionControl, DCSBiosOutputType.StringType);

        BIOSEventHandler.AttachStringListener(this);
    }

    public void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        if (_versionOutput == null || e.Address != _versionOutput.Address)
            return;

        var version = Sanitize(e.StringData);
        if (version.Length == 0)
            return;

        Action<string?>? handler;
        lock (_lock)
        {
            if (string.Equals(version, _version, StringComparison.Ordinal))
                return;

            _version = version;
            handler = VersionChanged;
        }

        handler?.Invoke(version);
    }

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
    }
}
