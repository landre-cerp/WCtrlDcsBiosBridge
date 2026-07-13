using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// Locates the DCS-BIOS "doc/json" folder under the user's actual Saved Games
/// location, resolved via the Windows Known Folder API. Saved Games can be
/// relocated by the user (right-click > Properties > Location), which moves it
/// away from the hardcoded %USERPROFILE%\Saved Games path — resolving it
/// through SHGetKnownFolderPath keeps auto-detection working after that move.
/// </summary>
public static class DcsBiosJsonLocationDetector
{
    private static readonly Guid SavedGamesFolderId = new("4C5C32FF-BB9D-43b0-B5B4-2D72E54EAAA4");

    // Checked in order; the plain "DCS" (stable release) install is the most common.
    private static readonly string[] DcsVariants =
    [
        "DCS",
        "DCS.openbeta",
        "DCS.server",
        "DCS.openbeta.server"
    ];

    /// <summary>
    /// Attempts to find an existing DCS-BIOS JSON folder under the resolved Saved
    /// Games location. Returns null if Saved Games can't be resolved or no
    /// DCS-BIOS installation is found under any known DCS variant folder.
    /// </summary>
    public static string? TryDetect()
    {
        var savedGames = GetSavedGamesPath();
        if (savedGames is null)
            return null;

        foreach (var variant in DcsVariants)
        {
            try
            {
                var candidate = Path.Combine(savedGames, variant, "Scripts", "DCS-BIOS", "doc", "json");
                if (Directory.Exists(candidate) && Directory.EnumerateFiles(candidate, "*.json").Any())
                    return candidate;
            }
            catch
            {
                // Ignore and continue checking other variants.
            }
        }

        return null;
    }

    private static string? GetSavedGamesPath()
    {
        try
        {
            if (SHGetKnownFolderPath(SavedGamesFolderId, 0, IntPtr.Zero, out var pathPtr) != 0)
                return null;

            try
            {
                return Marshal.PtrToStringUni(pathPtr);
            }
            finally
            {
                Marshal.FreeCoTaskMem(pathPtr);
            }
        }
        catch
        {
            return null;
        }
    }

    [DllImport("shell32.dll")]
    private static extern int SHGetKnownFolderPath(
        [MarshalAs(UnmanagedType.LPStruct)] Guid rfid,
        uint dwFlags,
        IntPtr hToken,
        out IntPtr pszPath);
}
