using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using NLog;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Listens for JSON UDP packets sent by wctrl-export.lua on port 31090 and fires
/// <see cref="DataReceived"/> for each successfully parsed packet.
///
/// One UDP port can only really be read by one socket at a time, but several listeners
/// may want the same feed at once — e.g. two CDUs both mapped to the F-14B(U). So this
/// is a process-wide singleton (<see cref="Shared"/>) rather than something each listener
/// owns: every subscriber attaches to the same socket instead of racing to bind their own.
/// </summary>
internal sealed class SimExportReceiver : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private static readonly Lazy<SimExportReceiver> _shared = new(() => new SimExportReceiver());

    /// <summary>
    /// The single instance every listener subscribes to. Call <see cref="EnsureStarted"/>
    /// before relying on it — construction alone does not bind the socket.
    /// </summary>
    public static SimExportReceiver Shared => _shared.Value;

    public const int DefaultPort = 31090;

    /// <summary>
    /// Envelope version this build understands. Must move in lockstep with
    /// PROTOCOL_VERSION in wctrl-export.lua: the check is an equality, so a script and an app
    /// from different releases talk past each other and show nothing.
    /// </summary>
    public const int SupportedVersion = 3;

    public event Action<SimExportData>? DataReceived;

    private readonly object _startLock = new();
    private UdpClient?   _udp;
    private Thread?      _thread;
    private volatile bool _running;
    private bool _loggedVersionMismatch;

    /// <summary>
    /// Binds the socket and starts the receive thread, unless another subscriber already
    /// did. Safe to call from every listener that wants the feed — only the first call
    /// actually binds; later ones are no-ops.
    /// </summary>
    public void EnsureStarted(int port = DefaultPort)
    {
        if (_running) return;

        lock (_startLock)
        {
            if (_running) return;

            // Built on a local and only published once it is listening. A bind that throws
            // has to leave the singleton exactly as it was, so the next subscriber's call is
            // a clean retry rather than one that overwrites — and leaks — this socket.
            var udp = new UdpClient();
            try
            {
                udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                udp.Client.Bind(new IPEndPoint(IPAddress.Any, port));
                udp.Client.ReceiveTimeout = 200;

                _udp     = udp;
                _running = true;
                _thread  = new Thread(() => ReceiveLoop(udp)) { IsBackground = true, Name = "SimExportReceiver" };
                _thread.Start();
            }
            catch
            {
                _running = false;
                _udp     = null;
                _thread  = null;
                udp.Dispose();
                throw;
            }

            Logger.Info($"SimExportReceiver started on UDP port {port}");
        }
    }

    public void Stop()
    {
        lock (_startLock)
        {
            _running = false;
            try { _udp?.Close(); } catch { /* ignore */ }
            _udp    = null;
            _thread = null;
        }
        Logger.Info("SimExportReceiver stopped");
    }

    public void Dispose() => Stop();

    // Takes the client it was started with rather than reading the field: Stop() clears
    // _udp, and the loop must not race it into a null dereference on the way out.
    private void ReceiveLoop(UdpClient udp)
    {
        var ep = new IPEndPoint(IPAddress.Any, 0);
        while (_running)
        {
            try
            {
                var bytes = udp.Receive(ref ep);
                var json  = Encoding.UTF8.GetString(bytes);
                var data  = JsonConvert.DeserializeObject<SimExportData>(json);
                if (data != null && AcceptVersion(data))
                    DataReceived?.Invoke(data);
            }
            catch (SocketException ex) when (ex.SocketErrorCode == SocketError.TimedOut)
            {
                // Normal: no data within 200 ms — keep looping
            }
            catch (SocketException)
            {
                // Socket closed during Stop() — exit cleanly
                break;
            }
            catch (ObjectDisposedException)
            {
                // Same, when Stop() won the race to dispose before the next Receive
                break;
            }
            catch (Exception ex)
            {
                Logger.Warn(ex, "SimExportReceiver: failed to parse packet");
            }
        }
    }

    // The lua script is installed into DCS by hand and updates independently of this
    // build, so a stale script is expected rather than exceptional. Drop the packet and
    // say so once, instead of silently reporting every field as absent.
    private bool AcceptVersion(SimExportData data)
    {
        if (data.Ver == SupportedVersion) return true;

        if (!_loggedVersionMismatch)
        {
            _loggedVersionMismatch = true;
            Logger.Warn($"Ignoring export packets: protocol version {data.Ver?.ToString() ?? "missing"}, " +
                        $"expected {SupportedVersion}. Update wctrl-export.lua in DCS Scripts.");
        }

        return false;
    }
}
