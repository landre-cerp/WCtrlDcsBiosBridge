using Newtonsoft.Json;
using NLog;
using System.IO;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge;

/// <summary>
/// Manages CDU and FrontPanel device detection and connection
/// </summary>
public class DeviceManager
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    /// <summary>
    /// Progress info for async device detection
    /// </summary>
    public sealed record DeviceDetectionProgress(int Current, int Total, string Message);

    /// <summary>
    /// Asynchronously detects and connects to all available CDU and FrontPanel devices with progress reporting
    /// </summary>
    /// <param name="resetDevices">
    /// Reset each device to its known-good state after connecting (devices keep their
    /// state across app restarts). Disable for SimApp Pro users who manage lighting externally.
    /// </param>
    public static async Task<List<DeviceInfo>> DetectAndConnectDevicesAsync(
        IProgress<DeviceDetectionProgress>? progress = null,
        CancellationToken cancellationToken = default,
        bool resetDevices = true)
    {
        var detectedDevices = new List<DeviceInfo>();
        try
        {
            cancellationToken.ThrowIfCancellationRequested();

            // Detect CDU devices
            var cduDeviceIdentifiers = await Task.Run(() => CduFactory.FindLocalDevices().ToList(), cancellationToken).ConfigureAwait(false);
            
            // Detect Frontpanel devices
            var frontpanelDeviceIdentifiers = await Task.Run(() => FrontpanelFactory.FindLocalDevices().ToList(), cancellationToken).ConfigureAwait(false);

            var totalDevices = cduDeviceIdentifiers.Count + frontpanelDeviceIdentifiers.Count;
            progress?.Report(new DeviceDetectionProgress(0, totalDevices, totalDevices == 0 ? "No devices found" : $"Found {totalDevices} device(s). Connecting..."));

            int currentIndex = 0;

            // Connect to CDU devices
            for (int i = 0; i < cduDeviceIdentifiers.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var deviceId = cduDeviceIdentifiers[i];
                progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Connecting CDU device {i + 1}/{cduDeviceIdentifiers.Count}..."));
                try
                {
                    var cdu = await Task.Run(() => CduFactory.ConnectLocal(deviceId), cancellationToken).ConfigureAwait(false);
                    InitializeCdu(cdu);
                    if (resetDevices) cdu.Reset();
                    var displayName = GetDeviceName(deviceId);
                    var deviceInfo = new DeviceInfo(cdu, deviceId, displayName);
                    detectedDevices.Add(deviceInfo);
                    currentIndex++;
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Connected {displayName} ({currentIndex}/{totalDevices})"));
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, $"Failed to connect to CDU device {i + 1}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Failed to connect CDU device {i + 1}: {ex.Message}"));
                    currentIndex++;
                }
            }

            // Connect to frontpanel devices
            for (int i = 0; i < frontpanelDeviceIdentifiers.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var deviceId = frontpanelDeviceIdentifiers[i];
                progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Connecting frontpanel device {i + 1}/{frontpanelDeviceIdentifiers.Count}..."));
                try
                {
                    Logger.Info($"About to connect frontpanel device: {deviceId.Description}");
                    var frontpanel = await Task.Run(() => FrontpanelFactory.ConnectLocal(deviceId), cancellationToken).ConfigureAwait(false);
                    
                    if (frontpanel == null)
                    {
                        Logger.Error($"FrontpanelFactory.ConnectLocal returned null for device {deviceId.Description}");
                        throw new InvalidOperationException($"Failed to connect to frontpanel device: ConnectLocal returned null");
                    }
                    
                    Logger.Info($"Frontpanel device connected: IsConnected={frontpanel.IsConnected}, Type={frontpanel.GetType().Name}");

                    if (resetDevices) frontpanel.Reset();

                    var displayName = GetDeviceName(deviceId);
                    var deviceInfo = new DeviceInfo(frontpanel, deviceId, displayName);
                    detectedDevices.Add(deviceInfo);
                    currentIndex++;
                    Logger.Info($"Successfully added frontpanel device: {displayName}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Connected {displayName} ({currentIndex}/{totalDevices})"));
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, $"Failed to connect to frontpanel device {i + 1}: {ex.Message}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, $"Failed to connect frontpanel device {i + 1}: {ex.Message}"));
                    currentIndex++;
                }
            }

            progress?.Report(new DeviceDetectionProgress(totalDevices, totalDevices, $"Detection complete. {detectedDevices.Count} connected."));
        }
        catch (OperationCanceledException)
        {
            progress?.Report(new DeviceDetectionProgress(0, 0, "Device detection cancelled"));
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to detect devices");
            progress?.Report(new DeviceDetectionProgress(0, 0, $"Detection error: {ex.Message}"));
        }
        return detectedDevices;
    }

    private static void InitializeCdu(ICdu mcdu)
    {
        // Load A-10C font as default for menu display
        // Will be replaced by aircraft-specific font when listener starts
        using var fileStream = new FileStream("resources/a10c-font-21x31.json", FileMode.Open, FileAccess.Read);
        using var reader = new StreamReader(fileStream);
        var fontJson = reader.ReadToEnd();
        mcdu.UseFont(JsonConvert.DeserializeObject<McduFontFile>(fontJson), true);
    }

    /// <summary>
    /// Gets a friendly name for a device
    /// </summary>
    public static string GetDeviceName(DeviceIdentifier deviceId) => deviceId.Description;

    /// <summary>
    /// Disposes a list of devices safely
    /// </summary>
    public static void DisposeDevices(IEnumerable<DeviceInfo> devices)
    {
        if (devices == null) return;
        foreach (var deviceInfo in devices)
        {
            try
            {
                deviceInfo.Dispose();
            }
            catch (Exception ex)
            {
                Logger.Warn(ex, "Error disposing device");
            }
        }
    }
}

