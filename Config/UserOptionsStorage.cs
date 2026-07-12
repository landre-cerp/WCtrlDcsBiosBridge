using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using WCtrlDcsBiosBridge.Common;

namespace WCtrlDcsBiosBridge.Config;

public static class UserOptionsStorage
{
    public static string ConfigFilePath => Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory,
        "useroptions.json");

    // Defaults now live on UserOptions and the per-aircraft option classes. Language is the
    // exception: on first run we detect the user's own language (see LanguageDetector) rather
    // than defaulting to System, since Windows' display language can differ from what the
    // user actually prefers (e.g. an English-language Windows install for a French user).
    public static UserOptions GetDefaultOptions() => new() { Language = LanguageDetector.DetectPreferredLanguage() };

    public static UserOptions Load()
    {
        var result = TryLoad();
        return result.IsSuccess ? result.Value! : GetDefaultOptions();
    }

    /// <summary>
    /// Attempts to load user options from the config file.
    /// Returns a Result indicating success or failure without throwing exceptions.
    /// </summary>
    /// <returns>A Result containing the loaded user options or an error message</returns>
    public static Result<UserOptions> TryLoad()
    {
        try
        {
            if (!File.Exists(ConfigFilePath))
            {
                var defaultOptions = GetDefaultOptions();
                var saveResult = TrySave(defaultOptions);
                if (!saveResult.IsSuccess)
                {
                    return Result<UserOptions>.Success(defaultOptions);
                }
                return Result<UserOptions>.Success(defaultOptions);
            }

            var json = File.ReadAllText(ConfigFilePath);

            // Migrate the old flat layout (per-aircraft settings at the root) to the
            // grouped layout once, preserving the user's customised keys, then re-save.
            var root = JsonNode.Parse(json);
            if (root?["A10C"] is null && root?["PerfPageKey"] is not null)
            {
                var migrated = JsonSerializer.Deserialize<LegacyUserOptions>(json)!.ToUserOptions();
                TrySave(migrated);
                return Result<UserOptions>.Success(migrated);
            }

            var options = JsonSerializer.Deserialize<UserOptions>(json);

            if (options is null)
                return Result<UserOptions>.Success(GetDefaultOptions());

            return Result<UserOptions>.Success(options);
        }
        catch (Exception ex)
        {
            return Result<UserOptions>.Failure($"Error loading user options: {ex.Message}");
        }
    }

    public static void Save(UserOptions? options)
    {
        TrySave(options);
    }

    /// <summary>
    /// Attempts to save user options to the config file.
    /// Returns a Result indicating success or failure without throwing exceptions.
    /// </summary>
    /// <param name="options">The user options to save</param>
    /// <returns>A Result indicating success or an error message</returns>
    public static Result<Unit> TrySave(UserOptions? options)
    {
        try
        {
            if (options is null)
                return Result<Unit>.Success(Unit.Value);

            var dir = Path.GetDirectoryName(ConfigFilePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            var json = JsonSerializer.Serialize(options, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(ConfigFilePath, json);

            return Result<Unit>.Success(Unit.Value);
        }
        catch (Exception ex)
        {
            return Result<Unit>.Failure($"Error saving user options: {ex.Message}");
        }
    }
}

/// <summary>
/// The pre-grouping (flat) layout of useroptions.json, used only to migrate older
/// files into <see cref="UserOptions"/>. Safe to delete once users have upgraded.
/// </summary>
internal sealed class LegacyUserOptions
{
    public bool DisplayBottomAligned { get; set; }
    public bool DisplayCMS { get; set; }
    public bool EnablePerfPages { get; set; } = true;
    public string PerfPageKey { get; set; } = "FuelPred";
    public bool ForwardCduKeysToSim { get; set; }
    public bool DisableLightingManagement { get; set; }
    public bool AutoStart { get; set; }
    public bool MinimizeOnStart { get; set; }
    public string NextPageKey { get; set; } = "NextPage";
    public string PrevPageKey { get; set; } = "PrevPage";
    public string F16CPrevDisplayKey { get; set; } = "PrevPage";
    public string F16CNextDisplayKey { get; set; } = "NextPage";
    public string F16CRwrDisplayKey { get; set; } = "LineSelectRight1";
    public string F14RioDisplayKey { get; set; } = "PrevPage";
    public string F14RadioDisplayKey { get; set; } = "NextPage";
    public ThemePreference Theme { get; set; } = ThemePreference.System;
    public bool CloseResetBacklight { get; set; } = true;
    public bool CloseResetMarkers { get; set; } = true;

    public UserOptions ToUserOptions() => new()
    {
        DisableLightingManagement = DisableLightingManagement,
        AutoStart = AutoStart,
        MinimizeOnStart = MinimizeOnStart,
        Theme = Theme,
        CloseResetBacklight = CloseResetBacklight,
        CloseResetMarkers = CloseResetMarkers,
        A10C = new A10COptions
        {
            DisplayBottomAligned = DisplayBottomAligned,
            DisplayCMS = DisplayCMS,
            EnablePerfPages = EnablePerfPages,
            PerfPageKey = PerfPageKey,
            NextPageKey = NextPageKey,
            PrevPageKey = PrevPageKey,
            ForwardCduKeysToSim = ForwardCduKeysToSim,
        },
        // FA-18C previously shared the A-10C NextPage/PrevPage keys; keep that value.
        FA18C = new FA18COptions { ShowIfeiKey = NextPageKey, ShowUfcKey = PrevPageKey },
        F16C = new F16COptions { DedKey = F16CPrevDisplayKey, NavKey = F16CNextDisplayKey, RwrKey = F16CRwrDisplayKey },
        F14 = new F14Options { RioKey = F14RioDisplayKey, RadioKey = F14RadioDisplayKey },
    };
}