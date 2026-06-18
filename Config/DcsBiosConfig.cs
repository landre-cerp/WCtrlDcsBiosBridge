using System;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// Exception thrown when configuration validation fails.
/// </summary>
public class ConfigException(string message) : Exception(message);

/// <summary>
/// Configuration class for DCS-BIOS connection settings.
/// </summary>
public class DcsBiosConfig
{
    /// <summary>
    /// IP address to receive UDP data from DCS-BIOS (multicast address).
    /// </summary>
    public string ReceiveFromIpUdp { get; set; } = "239.255.50.10";

    /// <summary>
    /// IP address to send UDP data to DCS-BIOS.
    /// </summary>
    public string SendToIpUdp { get; set; } = "127.0.0.1";

    /// <summary>
    /// Port number to receive UDP data from DCS-BIOS.
    /// </summary>
    public int ReceivePortUdp { get; set; } = 5010;

    /// <summary>
    /// Port number to send UDP data to DCS-BIOS.
    /// </summary>
    public int SendPortUdp { get; set; } = 7778;

    /// <summary>
    /// Path to the DCS-BIOS JSON files directory.
    /// </summary>
    public string DcsBiosJsonLocation { get; set; } = string.Empty;
}
