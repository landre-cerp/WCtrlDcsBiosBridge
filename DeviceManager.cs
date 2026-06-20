using Newtonsoft.Json;
using NLog;
using System.IO;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Devices;

namespace WCtrlDcsBiosBridge;

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
                    // No default/menu font is flashed here on purpose. Flashing a throwaway
                    // font during detection and then the aircraft font moments later corrupts
                    // the device's glyph RAM (garbled glyphs, only cleared by a USB replug).
                    // The aircraft font flashed by the listener is the single font upload.
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

    /// <summary>
    /// Gets a friendly name for a device
    /// </summary>
    public static string GetDeviceName(DeviceIdentifier deviceId) => deviceId.Description;

    /// <summary>
    /// Disposes a list of devices safely, applying the close-reset options before
    /// closing each USB connection. Pass <see langword="null"/> to skip the reset
    /// (e.g. when lighting management is disabled).
    /// </summary>
    internal static void DisposeDevices(IEnumerable<DeviceInfo> devices, CloseResetOptions? resetOptions = null)
    {
        if (devices == null) return;
        foreach (var deviceInfo in devices)
        {
            if (resetOptions != null)
            {
                try
                {
                    if (deviceInfo.Cdu != null)
                    {
                        var b = resetOptions.BrightnessPercent;
                        deviceInfo.Cdu.Cleanup(b, b, resetOptions.Markers ? 0 : 100);
                    }
                    if (deviceInfo.Frontpanel != null)
                    {
                        deviceInfo.Frontpanel.Reset();
                        var bv = resetOptions.BrightnessByte;
                        deviceInfo.Frontpanel.SetBrightness(bv, bv, bv);
                    }
                }
                catch (Exception ex)
                {
                    Logger.Warn(ex, "Error resetting device during dispose");
                }
            }

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

