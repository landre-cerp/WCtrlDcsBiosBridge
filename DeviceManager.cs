using Newtonsoft.Json;
using NLog;
using System.IO;
using System.Linq;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Common;
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
            progress?.Report(new DeviceDetectionProgress(0, totalDevices, totalDevices == 0
                ? Strings.Get("Device_NoDevicesFound")
                : Strings.Format("Device_FoundDevicesConnectingFormat", totalDevices)));

            int currentIndex = 0;

            // Connect to CDU devices
            for (int i = 0; i < cduDeviceIdentifiers.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var deviceId = cduDeviceIdentifiers[i];
                progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_ConnectingCduFormat", i + 1, cduDeviceIdentifiers.Count)));
                try
                {
                    var deviceInfo = await ConnectDeviceAsync(deviceId, resetDevices, cancellationToken).ConfigureAwait(false)
                        ?? throw new InvalidOperationException("Failed to connect to CDU device: ConnectLocal returned null");
                    detectedDevices.Add(deviceInfo);
                    currentIndex++;
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_ConnectedFormat", deviceInfo.DisplayName, currentIndex, totalDevices)));
                }
                catch (OperationCanceledException)
                {
                    // Cancellation is not a per-device failure — let it abort the whole detect.
                    throw;
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, $"Failed to connect to CDU device {i + 1}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_FailedConnectCduFormat", i + 1, ex.Message)));
                    currentIndex++;
                }
            }

            // Connect to frontpanel devices
            for (int i = 0; i < frontpanelDeviceIdentifiers.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var deviceId = frontpanelDeviceIdentifiers[i];
                progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_ConnectingFrontpanelFormat", i + 1, frontpanelDeviceIdentifiers.Count)));
                try
                {
                    Logger.Info($"About to connect frontpanel device: {deviceId.Description}");
                    var deviceInfo = await ConnectDeviceAsync(deviceId, resetDevices, cancellationToken).ConfigureAwait(false)
                        ?? throw new InvalidOperationException("Failed to connect to frontpanel device: ConnectLocal returned null");
                    detectedDevices.Add(deviceInfo);
                    currentIndex++;
                    Logger.Info($"Successfully added frontpanel device: {deviceInfo.DisplayName}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_ConnectedFormat", deviceInfo.DisplayName, currentIndex, totalDevices)));
                }
                catch (OperationCanceledException)
                {
                    // Cancellation is not a per-device failure — let it abort the whole detect.
                    throw;
                }
                catch (Exception ex)
                {
                    Logger.Error(ex, $"Failed to connect to frontpanel device {i + 1}: {ex.Message}");
                    progress?.Report(new DeviceDetectionProgress(currentIndex, totalDevices, Strings.Format("Device_FailedConnectFrontpanelFormat", i + 1, ex.Message)));
                    currentIndex++;
                }
            }

            progress?.Report(new DeviceDetectionProgress(totalDevices, totalDevices, Strings.Format("Device_DetectionCompleteFormat", detectedDevices.Count)));
        }
        catch (OperationCanceledException)
        {
            // Propagate cancellation so the caller can abort the post-detection pipeline
            // (UI rebuild, watcher startup, auto-start) instead of treating a cancelled
            // run as a completed one with a partial device list.
            progress?.Report(new DeviceDetectionProgress(0, 0, Strings.Get("Status_DetectionCancelled")));
            throw;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to detect devices");
            progress?.Report(new DeviceDetectionProgress(0, 0, Strings.Format("Device_DetectionErrorFormat", ex.Message)));
        }
        return detectedDevices;
    }

    /// <summary>
    /// Connects to a single device (CDU or frontpanel) identified by <paramref name="deviceId"/>.
    /// Returns the wrapped <see cref="DeviceInfo"/>, or <see langword="null"/> if the
    /// underlying factory could not produce a device. Connection failures (e.g. the USB
    /// stream cannot be opened) propagate as exceptions for the caller to handle.
    /// </summary>
    /// <param name="resetDevices">
    /// Reset the device to its known-good state after connecting. No throwaway font is ever
    /// flashed here: flashing a menu font now and the aircraft font moments later corrupts
    /// the device's glyph RAM. The aircraft font flashed by the listener is the single upload.
    /// </param>
    public static async Task<DeviceInfo?> ConnectDeviceAsync(
        DeviceIdentifier deviceId,
        bool resetDevices = true,
        CancellationToken cancellationToken = default)
    {
        var displayName = GetDeviceName(deviceId);

        if (IsCdu(deviceId))
        {
            var cdu = await Task.Run(() => CduFactory.ConnectLocal(deviceId), cancellationToken).ConfigureAwait(false);
            if (cdu == null) return null;
            if (resetDevices) cdu.Reset();
            return new DeviceInfo(cdu, deviceId, displayName);
        }

        var frontpanel = await Task.Run(() => FrontpanelFactory.ConnectLocal(deviceId), cancellationToken).ConfigureAwait(false);
        if (frontpanel == null) return null;
        if (resetDevices) frontpanel.Reset();
        return new DeviceInfo(frontpanel, deviceId, displayName);
    }

    /// <summary>
    /// Returns the identifiers of every supported CDU and frontpanel currently present on
    /// the local machine. Used by the device watcher to diff arrivals and removals.
    /// </summary>
    public static IReadOnlyList<DeviceIdentifier> FindLocalSupportedDevices()
    {
        var result = new List<DeviceIdentifier>();
        result.AddRange(CduFactory.FindLocalDevices());
        result.AddRange(FrontpanelFactory.FindLocalDevices());
        return result;
    }

    /// <summary>
    /// True if the identifier denotes a CDU device (as opposed to a frontpanel).
    /// </summary>
    public static bool IsCdu(DeviceIdentifier deviceId) =>
        SupportedDevices.AllSupportedDevices.Any(d => d.Equals(deviceId));

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

