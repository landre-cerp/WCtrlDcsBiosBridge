using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Text.Json;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Common;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// Manages configuration loading, saving, and validation for the DCS-BIOS bridge application.
/// </summary>
public static class ConfigManager
{
    private static readonly string ConfigFile =
        Path.Combine(AppContext.BaseDirectory, "config.json");

    /// <summary>
    /// Loads the configuration from the config file, creating a default one if it doesn't exist.
    /// Uses the Result pattern internally.
    /// </summary>
    /// <returns>The loaded and validated configuration</returns>
    /// <exception cref="ConfigException">Thrown when configuration is invalid or missing</exception>
    public static DcsBiosConfig Load()
    {
        var result = TryLoad();
        if (!result.IsSuccess)
            throw new ConfigException(result.Error!);
        return result.Value!;
    }

    /// <summary>
    /// Attempts to load the configuration from the config file, creating a default one if it doesn't exist.
    /// Returns a Result indicating success or failure without throwing exceptions.
    /// </summary>
    /// <returns>A Result containing the loaded configuration or an error message</returns>
    public static Result<DcsBiosConfig> TryLoad()
    {
        try
        {
            if (!File.Exists(ConfigFile))
            {
                var detectedLocation = DcsBiosJsonLocationDetector.TryDetect();
                var defaultConfig = new DcsBiosConfig
                {
                    DcsBiosJsonLocation = detectedLocation ?? string.Empty
                };
                var saveResult = TrySave(defaultConfig);
                if (!saveResult.IsSuccess)
                    return Result<DcsBiosConfig>.Failure($"Could not create default config: {saveResult.Error}");

                // A detected location has already been confirmed to exist and contain
                // DCS-BIOS JSON files, so the config is usable as-is — no need to force
                // the first-run review prompt.
                if (detectedLocation is not null && TryValidate(defaultConfig).IsSuccess)
                    return Result<DcsBiosConfig>.Success(defaultConfig);

                return Result<DcsBiosConfig>.Failure($"Configuration file not found. A default config has been created at {ConfigFile}. Please review and update it as necessary.");
            }

            var json = File.ReadAllText(ConfigFile);
            var config = JsonSerializer.Deserialize<DcsBiosConfig>(json);
            
            if (config is null)
                return Result<DcsBiosConfig>.Failure("Failed to deserialize configuration file.");

            var validationResult = TryValidate(config);
            if (!validationResult.IsSuccess)
                return Result<DcsBiosConfig>.Failure(validationResult.Error!);

            return Result<DcsBiosConfig>.Success(config);
        }
        catch (Exception ex)
        {
            return Result<DcsBiosConfig>.Failure($"Error loading configuration: {ex.Message}");
        }
    }

    /// <summary>
    /// Saves the configuration to the config file.
    /// </summary>
    /// <param name="config">The configuration to save</param>
    public static void Save(DcsBiosConfig config)
    {
        var result = TrySave(config);
        if (!result.IsSuccess)
            throw new ConfigException(result.Error!);
    }

    /// <summary>
    /// Attempts to save the configuration to the config file.
    /// Returns a Result indicating success or failure without throwing exceptions.
    /// </summary>
    /// <param name="config">The configuration to save</param>
    /// <returns>A Result indicating success or an error message</returns>
    public static Result<Unit> TrySave(DcsBiosConfig config)
    {
        try
        {
            var json = JsonSerializer.Serialize(config, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(ConfigFile, json);
            return Result<Unit>.Success(Unit.Value);
        }
        catch (Exception ex)
        {
            return Result<Unit>.Failure($"Error saving configuration: {ex.Message}");
        }
    }

    /// <summary>
    /// Public validation entry point for UI and callers.
    /// Throws ConfigException if invalid.
    /// </summary>
    public static void Validate(DcsBiosConfig config)
    {
        var result = TryValidate(config);
        if (!result.IsSuccess)
            throw new ConfigException(result.Error!);
    }

    /// <summary>
    /// Attempts to validate the configuration settings.
    /// Returns a Result indicating success or failure without throwing exceptions.
    /// </summary>
    /// <param name="config">The configuration to validate</param>
    /// <returns>A Result indicating success or an error message</returns>
    public static Result<Unit> TryValidate(DcsBiosConfig config)
    {
        if (string.IsNullOrWhiteSpace(config.DcsBiosJsonLocation))
        {
            return Result<Unit>.Failure("The 'DcsBiosJsonLocation' field in the configuration file cannot be empty. Please specify a valid path to the DCS-BIOS JSON file(s).");
        }

        if (!Directory.Exists(config.DcsBiosJsonLocation))
        {
            return Result<Unit>.Failure($"The folder specified by 'DcsBiosJsonLocation' does not exist: {config.DcsBiosJsonLocation}");
        }

        if (config.ReceivePortUdp < 1 || config.ReceivePortUdp > 65535)
        {
            return Result<Unit>.Failure("ReceivePortUdp must be between 1 and 65535.");
        }
        if (config.SendPortUdp < 1 || config.SendPortUdp > 65535)
        {
            return Result<Unit>.Failure("SendPortUdp must be between 1 and 65535.");
        }
        if (!IPAddress.TryParse(config.ReceiveFromIpUdp, out var receiveIp))
        {
            return Result<Unit>.Failure("ReceiveFromIpUdp is not a valid IP address.");
        }
        if (!IPAddress.TryParse(config.SendToIpUdp, out var sendIp))
        {
            return Result<Unit>.Failure("SendToIpUdp is not a valid IP address.");
        }
        if (receiveIp.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork)
        {
            return Result<Unit>.Failure("ReceiveFromIpUdp must be an IPv4 address.");
        }
        if (sendIp.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork)
        {
            return Result<Unit>.Failure("SendToIpUdp must be an IPv4 address.");
        }

        return Result<Unit>.Success(Unit.Value);
    }

    /// <summary>
    /// Returns the list of expected DCS-BIOS JSON files that are missing under the provided directory.
    /// This is a soft check (no exceptions thrown).
    /// </summary>
    public static IReadOnlyList<string> GetMissingExpectedJsonFiles(string rootDirectory)
    {
        if (string.IsNullOrWhiteSpace(rootDirectory) || !Directory.Exists(rootDirectory))
            return Array.Empty<string>();

        var present = new HashSet<string>(
            Directory.EnumerateFiles(rootDirectory, "*.json", SearchOption.AllDirectories)
                     .Select(f => Path.GetFileName(f)!.ToLowerInvariant())
        );

        return AircraftRegistry.ExpectedJsonFiles
            .Where(name => !present.Contains(name.ToLowerInvariant()))
            .ToArray();
    }
}
