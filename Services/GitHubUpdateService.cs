using System.Net.Http;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace WWCduDcsBiosBridge.Services;

public enum UpdateChannel
{
    Stable,
    Prerelease
}

public sealed class GitHubUpdateService
{
    private readonly string owner;
    private readonly string repo;
    private readonly HttpClient http;

    public GitHubUpdateService(string owner, string repo, HttpClient? httpClient = null)
    {
        this.owner = owner;
        this.repo = repo;
        http = httpClient ?? new HttpClient { Timeout = TimeSpan.FromSeconds(5) };
        http.DefaultRequestHeaders.UserAgent.ParseAdd($"{repo}/UpdateCheck");
        http.DefaultRequestHeaders.Accept.ParseAdd("application/vnd.github+json");
        http.DefaultRequestHeaders.Add("X-GitHub-Api-Version", "2022-11-28");
    }

    public async Task<UpdateCheckResult?> CheckForUpdatesAsync(string currentVersion, UpdateChannel channel = UpdateChannel.Stable, CancellationToken ct = default)
    {
        var latest = await GetLatestReleaseAsync(channel, ct);
        if (latest is null || string.IsNullOrEmpty(latest.TagName)) return null;

        var hasUpdate = CompareSemVer(latest.TagName, currentVersion) > 0;
        return new UpdateCheckResult(hasUpdate, latest.TagName, latest.HtmlUrl);
    }

    private async Task<GitHubRelease?> GetLatestReleaseAsync(UpdateChannel channel, CancellationToken ct)
    {
        if (channel == UpdateChannel.Stable)
        {
            // Prefer the official "latest" stable endpoint
            var latestUrl = $"https://api.github.com/repos/{owner}/{repo}/releases/latest";
            try
            {
                using var latestStream = await http.GetStreamAsync(latestUrl, ct);
                var latest = await JsonSerializer.DeserializeAsync<GitHubRelease>(latestStream, cancellationToken: ct);
                if (latest is { Draft: false, Prerelease: false, TagName: { Length: > 0 } })
                    return latest;
            }
            catch
            {
                // fall through
            }
        }

        // Fallback to list releases and pick by channel with SemVer ordering
        var listUrl = $"https://api.github.com/repos/{owner}/{repo}/releases?per_page=30";
        try
        {
            using var listStream = await http.GetStreamAsync(listUrl, ct);
            var all = await JsonSerializer.DeserializeAsync<List<GitHubRelease>>(listStream, cancellationToken: ct);
            if (all is null) return null;

            if (channel == UpdateChannel.Prerelease)
            {
                var pre = all.Where(r => r is { Draft: false, Prerelease: true, TagName.Length: > 0 })
                             .OrderByDescending(r => r.TagName, Comparer<string>.Create(CompareSemVer))
                             .FirstOrDefault();
                if (pre is not null) return pre;
                // If no prereleases exist, fall back to highest stable
            }

            return all.Where(r => r is { Draft: false, Prerelease: false, TagName.Length: > 0 })
                      .OrderByDescending(r => r.TagName, Comparer<string>.Create(CompareSemVer))
                      .FirstOrDefault();
        }
        catch
        {
            return null;
        }
    }

    // SemVer parsing and comparison (handles prerelease correctly)
    private static int CompareSemVer(string a, string b)
    {
        var pa = ParseSemVer(a);
        var pb = ParseSemVer(b);

        var coreCmp = pa.Core.CompareTo(pb.Core);
        if (coreCmp != 0) return coreCmp;

        var aPre = pa.PreRelease;
        var bPre = pb.PreRelease;

        if (string.IsNullOrEmpty(aPre) && string.IsNullOrEmpty(bPre)) return 0;
        if (string.IsNullOrEmpty(aPre)) return 1;   // release > prerelease
        if (string.IsNullOrEmpty(bPre)) return -1;  // prerelease < release
        return ComparePrerelease(aPre, bPre);
    }

    private static int ComparePrerelease(string a, string b)
    {
        var aParts = a.Split('.', StringSplitOptions.RemoveEmptyEntries);
        var bParts = b.Split('.', StringSplitOptions.RemoveEmptyEntries);
        var len = Math.Max(aParts.Length, bParts.Length);

        for (int i = 0; i < len; i++)
        {
            var ai = i < aParts.Length ? aParts[i] : null;
            var bi = i < bParts.Length ? bParts[i] : null;

            if (ai == bi) continue;
            if (ai is null) return -1; // shorter has lower precedence
            if (bi is null) return 1;

            var aIsNum = int.TryParse(ai, out var aNum);
            var bIsNum = int.TryParse(bi, out var bNum);

            if (aIsNum && bIsNum)
            {
                var cmp = aNum.CompareTo(bNum);
                if (cmp != 0) return cmp;
            }
            else if (aIsNum != bIsNum)
            {
                // Numeric identifiers have lower precedence than non-numeric
                return aIsNum ? -1 : 1;
            }
            else
            {
                var cmp = string.CompareOrdinal(ai, bi);
                if (cmp != 0) return cmp;
            }
        }
        return 0;
    }

    private static (Version Core, string? PreRelease) ParseSemVer(string? tag)
    {
        if (string.IsNullOrWhiteSpace(tag)) return (new Version(0, 0, 0, 0), null);
        var t = tag.Trim();
        if (t.StartsWith("v", StringComparison.OrdinalIgnoreCase)) t = t[1..];

        // strip build metadata
        var plus = t.IndexOf('+');
        if (plus >= 0) t = t[..plus];

        string? pre = null;
        var dash = t.IndexOf('-');
        if (dash >= 0)
        {
            pre = t[(dash + 1)..];
            t = t[..dash];
        }

        // Normalize core to X.Y[.Z]
        if (t.Count(c => c == '.') == 0) t += ".0";
        if (!Version.TryParse(t, out var core))
            core = new Version(0, 0, 0, 0);

        return (core, pre);
    }

    private sealed class GitHubRelease
    {
        [JsonPropertyName("tag_name")] public string TagName { get; set; } = string.Empty;
        [JsonPropertyName("html_url")] public string HtmlUrl { get; set; } = string.Empty;
        [JsonPropertyName("published_at")] public DateTimeOffset? PublishedAt { get; set; }
        [JsonPropertyName("draft")] public bool Draft { get; set; }
        [JsonPropertyName("prerelease")] public bool Prerelease { get; set; }
    }
}

public sealed record UpdateCheckResult(bool HasUpdate, string? LatestTag, string? HtmlUrl);
