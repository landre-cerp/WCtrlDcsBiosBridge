using System.Reflection;

namespace WWCduDcsBiosBridge.Services;

public static class AppVersionProvider
{
    public static string GetAppVersion()
    {
        var asm = Assembly.GetExecutingAssembly();
        var info = asm.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion;
        var ver = string.IsNullOrWhiteSpace(info) ? asm.GetName().Version?.ToString() : info;
        if (string.IsNullOrWhiteSpace(ver)) return "0.0.0";

        var t = ver.Trim();
        if (t.StartsWith("v", StringComparison.OrdinalIgnoreCase)) t = t[1..];

        // Drop build metadata (+...), keep prerelease (-...)
        var plus = t.IndexOf('+');
        if (plus >= 0) t = t[..plus];

        return t;
    }

    public static bool IsPreRelease(string? version = null)
    {
        version ??= GetAppVersion();
        return version?.Contains('-') == true;
    }
}
