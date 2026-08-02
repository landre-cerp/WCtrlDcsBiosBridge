using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using NLog;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Listens for JSON UDP packets sent by wctrl-export.lua on port 31090 and fires
/// <see cref="DataReceived"/> for each successfully parsed packet.
/// </summary>
internal sealed class SimExportReceiver : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    public const int DefaultPort = 31090;

    /// <summary>
    /// Envelope version this build understands. Must move in lockstep with
    /// PROTOCOL_VERSION in wctrl-export.lua: the check is an equality, so a script and an app
    /// from different releases talk past each other and show nothing.
    /// </summary>
    public const int SupportedVersion = 2;

    public event Action<SimExportData>? DataReceived;

    private UdpClient?   _udp;
    private Thread?      _thread;
    private volatile bool _running;
    private bool _loggedVersionMismatch;

    public void Start(int port = DefaultPort)
    {
        _udp = new UdpClient();
        _udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        _udp.Client.Bind(new IPEndPoint(IPAddress.Any, port));
        _udp.Client.ReceiveTimeout = 200;

        _running = true;
        _thread  = new Thread(ReceiveLoop) { IsBackground = true, Name = "SimExportReceiver" };
        _thread.Start();
        Logger.Info($"SimExportReceiver started on UDP port {port}");
    }

    public void Stop()
    {
        _running = false;
        try { _udp?.Close(); } catch { /* ignore */ }
        Logger.Info("SimExportReceiver stopped");
    }

    public void Dispose() => Stop();

    private void ReceiveLoop()
    {
        var ep = new IPEndPoint(IPAddress.Any, 0);
        while (_running)
        {
            try
            {
                var bytes = _udp!.Receive(ref ep);
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
