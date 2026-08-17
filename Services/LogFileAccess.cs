using NLog;
using NLog.Targets;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// Reads the tail of the application log for the in-app viewer.
///
/// The name is asked of NLog rather than hardcoded, so editing NLog.config cannot silently
/// leave the viewer pointing at a file nobody writes any more.
/// </summary>
internal static class LogFileAccess
{
    /// <summary>How many lines of the tail the viewer shows.</summary>
    public const int DefaultMaxLines = 500;

    /// <summary>
    /// Absolute path of the file target named "logfile" in NLog.config, or null when the
    /// configuration never loaded or has no such target.
    /// </summary>
    public static string? ResolvePath()
    {
        var target = LogManager.Configuration?.FindTargetByName<FileTarget>("logfile");
        var rendered = target?.FileName?.Render(LogEventInfo.CreateNullEvent());

        return string.IsNullOrWhiteSpace(rendered)
            ? null
            : ResolveAgainstBase(rendered, AppContext.BaseDirectory);
    }

    /// <summary>
    /// Turns the rendered <c>fileName</c> into an absolute path the way NLog itself does.
    ///
    /// NLog.config gives the target a bare relative name ("log.txt"), and NLog's FilePathLayout
    /// resolves a relative name against the *application base directory*, not the process
    /// working directory — deliberately, because the working directory moves. Verified by
    /// launching the app with a different working directory: the log still landed beside the
    /// executable. So <c>Path.GetFullPath</c> alone is wrong here; it would resolve against the
    /// working directory and point at a file that does not exist whenever the app is started
    /// from somewhere else — a shortcut with a "Start in" field, or a launch from another
    /// process — which is precisely the case this has to get right.
    ///
    /// An already-absolute name (someone edits the config to use ${specialfolder} or similar)
    /// passes through untouched, which is what Path.Combine does with a rooted second argument.
    /// </summary>
    public static string ResolveAgainstBase(string renderedFileName, string baseDirectory) =>
        Path.GetFullPath(Path.Combine(baseDirectory, renderedFileName));

    /// <summary>
    /// Last <paramref name="maxLines"/> lines of the log. Throws whatever the file system
    /// throws — the caller shows the message rather than hiding an unreadable log behind an
    /// empty panel.
    /// </summary>
    public static string ReadTail(string path, int maxLines = DefaultMaxLines)
    {
        // The viewer is opened right after the failure it is meant to explain, so the lines
        // that matter are the ones still sitting in NLog's buffers.
        LogManager.Flush();

        // NLog keeps its own handle on the file open, so anything less permissive than
        // ReadWrite here fails with a sharing violation on the app's own log.
        using var stream = new FileStream(path, FileMode.Open, FileAccess.Read,
                                          FileShare.ReadWrite | FileShare.Delete);
        using var reader = new StreamReader(stream);

        return TailLines(reader, maxLines);
    }

    /// <summary>
    /// Last <paramref name="maxLines"/> lines of <paramref name="reader"/>, joined.
    ///
    /// Streams the whole thing through a bounded queue instead of seeking backwards from the
    /// end: linear in the file's length, but constant in memory, which is the side that matters
    /// here — NLog.config sets no archive or rolling policy, so log.txt grows for the life of
    /// the install and reading it whole is exactly what must not happen.
    /// </summary>
    public static string TailLines(TextReader reader, int maxLines)
    {
        if (maxLines <= 0) return string.Empty;

        var tail = new Queue<string>(maxLines);

        while (reader.ReadLine() is { } line)
        {
            if (tail.Count == maxLines) tail.Dequeue();
            tail.Enqueue(line);
        }

        return string.Join(Environment.NewLine, tail);
    }
}
