using System;
using System.IO;
using System.Text.Json;

namespace WWCduDcsBiosBridge.Config;

public static class UserOptionsStorage
{
    public static string ConfigFilePath => Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory,
        "useroptions.json");

    public static UserOptions GetDefaultOptions()
    {
        return new UserOptions
        {
            DisplayBottomAligned = false,
            DisplayCMS = false,
            DisableLightingManagement = false,
            Ch47CduSwitchWithSeat = false,
            AutoStart = false,
            MinimizeOnStart = false,
            NextPageKey = "NextPage",
            PrevPageKey = "PrevPage"
        };
    }

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