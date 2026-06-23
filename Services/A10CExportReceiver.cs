using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using NLog;
using WCtrlDcsBiosBridge.Aircrafts;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Listens for JSON UDP packets sent by a10c-calc.lua on port 31090 and fires
/// <see cref="DataReceived"/> for each successfully parsed packet.
/// </summary>
internal sealed class A10CExportReceiver : IDisposable
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    public const int DefaultPort = 31090;

    public event Action<A10CExportData>? DataReceived;

    private UdpClient?   _udp;
    private Thread?      _thread;
    private volatile bool _running;

    public void Start(int port = DefaultPort)
    {
        _udp = new UdpClient();
        _udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        _udp.Client.Bind(new IPEndPoint(IPAddress.Any, port));
        _udp.Client.ReceiveTimeout = 200;

        _running = true;
        _thread  = new Thread(ReceiveLoop) { IsBackground = true, Name = "A10CExportReceiver" };
        _thread.Start();
        Logger.Info($"A10CExportReceiver started on UDP port {port}");
    }

    public void Stop()
    {
        _running = false;
        try { _udp?.Close(); } catch { /* ignore */ }
        Logger.Info("A10CExportReceiver stopped");
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
                var data  = JsonConvert.DeserializeObject<A10CExportData>(json);
                if (data != null)
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
                Logger.Warn(ex, "A10CExportReceiver: failed to parse packet");
            }
        }
    }
}
