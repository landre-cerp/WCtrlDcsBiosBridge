using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Common;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// Reads and writes ledmappings.json. Kept apart from useroptions.json on purpose: a LED
/// mapping is worth sharing (an "A-10C on the PAP3" file can be posted and dropped in), and
/// that only works when the file holds nothing else.
/// </summary>
public static class LedMappingStorage
{
    public const string FileName = "ledmappings.json";

    public static string FilePath => Path.Combine(AppDomain.CurrentDomain.BaseDirectory, FileName);

    private static JsonSerializerOptions SerializerOptions => new()
    {
        WriteIndented = true,
        PropertyNameCaseInsensitive = true,
        // Enums as names: the file is meant to be read, edited and shared by hand.
        Converters = { new JsonStringEnumConverter(allowIntegerValues: false) },
    };

    /// <summary>
    /// Loads the mapping file. A missing file is success with an empty mapping — unlike
    /// useroptions.json this one is optional and is not written until the user binds something.
    /// </summary>
    public static Result<LedMappingFile> TryLoad(string? path = null)
    {
        var file = path ?? FilePath;
        try
        {
            if (!File.Exists(file))
                return Result<LedMappingFile>.Success(new LedMappingFile());

            var json = File.ReadAllText(file);
            var mapping = JsonSerializer.Deserialize<LedMappingFile>(json, SerializerOptions);

            return Result<LedMappingFile>.Success(mapping ?? new LedMappingFile());
        }
        catch (Exception ex)
        {
            return Result<LedMappingFile>.Failure($"Error loading LED mappings: {ex.Message}");
        }
    }

    public static Result<Unit> TrySave(LedMappingFile? mapping, string? path = null)
    {
        var file = path ?? FilePath;
        try
        {
            if (mapping is null)
                return Result<Unit>.Success(Unit.Value);

            var dir = Path.GetDirectoryName(file);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            File.WriteAllText(file, JsonSerializer.Serialize(mapping, SerializerOptions));
            return Result<Unit>.Success(Unit.Value);
        }
        catch (Exception ex)
        {
            return Result<Unit>.Failure($"Error saving LED mappings: {ex.Message}");
        }
    }
}

/// <summary>
/// The mapping the app is currently running on. A static holder rather than a constructor
/// argument: aircraft listeners are built by twelve factory lambdas that all take
/// <c>UserOptions</c> only, and the bindings are read in one place — the base listener.
/// </summary>
public static class LedMappingStore
{
    private static volatile LedMappingFile _current = new();

    /// <summary>The sanitized mapping in force. Never null.</summary>
    public static LedMappingFile Current => _current;

    /// <summary>Reloads from disk, dropping unusable entries and logging why.</summary>
    public static void Reload()
    {
        var result = LedMappingStorage.TryLoad();
        if (!result.IsSuccess)
        {
            App.Logger.Warn($"LED mappings not loaded: {result.Error}");
            return;
        }

        var clean = Apply(result.Value!);
        App.Logger.Info($"LED mappings loaded: {clean.Bindings.Count} binding(s)");
    }

    /// <summary>Sanitizes <paramref name="mapping"/>, writes it to disk and adopts it.</summary>
    public static Result<Unit> Save(LedMappingFile mapping)
    {
        var clean = Apply(mapping);
        var saved = LedMappingStorage.TrySave(clean);
        if (!saved.IsSuccess) App.Logger.Warn(saved.Error);
        return saved;
    }

    /// <summary>The bindings that apply to one aircraft, by registry display name.</summary>
    public static IReadOnlyList<LedBinding> ForAircraft(string aircraftDisplayName) =>
        _current.ForAircraft(aircraftDisplayName);

    private static LedMappingFile Apply(LedMappingFile mapping)
    {
        var (clean, warnings) = mapping.Sanitize(AircraftRegistry.All.Select(d => d.DisplayName));
        foreach (var warning in warnings) App.Logger.Warn($"LED mapping: {warning}");

        _current = clean;
        return clean;
    }
}
