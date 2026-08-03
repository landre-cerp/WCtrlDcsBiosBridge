using Newtonsoft.Json;
using WCtrlDcsBiosBridge.Aircrafts.C130J;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Packets captured from a running C-130J and replayed through the real export script, paired
/// with the shipped page schema. Everything about the CNI layout is derived from files outside
/// DCS, so these captures are the only thing that says the derivation is right.
/// </summary>
internal static class CniFixtures
{
    private static readonly Lazy<CniSchema> LazySchema = new(() =>
        CniSchema.Load(Path.Combine(AppContext.BaseDirectory, "Resources", "c130j-cni-pages.json")));

    public static CniSchema Schema => LazySchema.Value;

    private static string Dir => Path.Combine(AppContext.BaseDirectory, "Fixtures", "Cni");

    public static CniData Load(string name)
    {
        var path = Path.Combine(Dir, name + ".json");
        var packet = JsonConvert.DeserializeObject<SimExportData>(File.ReadAllText(path))
                     ?? throw new InvalidDataException($"unreadable fixture: {path}");
        return packet.Cni ?? throw new InvalidDataException($"fixture carries no cni: {path}");
    }

    public static IEnumerable<string> Names =>
        Directory.EnumerateFiles(Dir, "*.json")
                 .Select(Path.GetFileNameWithoutExtension)
                 .Where(n => n != null)
                 .Select(n => n!)
                 .OrderBy(n => n, StringComparer.Ordinal);

}
