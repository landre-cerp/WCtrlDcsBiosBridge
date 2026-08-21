using System.Text.RegularExpressions;
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
    /// Start of a log entry: the "${longdate}|${level:uppercase=true}|" prefix NLog.config
    /// writes. Anything that does not match is a continuation of the entry above it — the
    /// "${exception:format=tostring}" tail of a message is a whole stack trace, several
    /// physical lines of it.
    /// </summary>
    private static readonly Regex EntryHeader = new(
        @"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+\|(?<level>[A-Z]+)\|",
        RegexOptions.Compiled);

    /// <summary>Levels the "errors only" filter keeps. FATAL is in: it is the worst line the
    /// log can hold, and a filter named "errors" that hides it would be a trap.</summary>
    private static readonly HashSet<string> ErrorLevels = new(StringComparer.Ordinal)
    {
        "ERROR", "FATAL"
    };

    /// <summary>
    /// The ERROR and FATAL entries of <paramref name="text"/>, each with its continuation
    /// lines, in order; empty when there are none.
    ///
    /// Filters whole entries rather than matching lines, because the line the user opened the
    /// viewer for is rarely the header — it is the stack frame three lines below it, which
    /// carries no level of its own and would be dropped by a per-line grep.
    ///
    /// Lines before the first header are dropped: <see cref="TailLines"/> cuts at a line
    /// boundary, not an entry boundary, so a tail can open midway through an entry whose level
    /// was never read. Keeping those would be guessing.
    /// </summary>
    public static string FilterErrors(string text)
    {
        if (string.IsNullOrEmpty(text)) return string.Empty;

        var kept = new List<string>();
        var keeping = false;

        foreach (var line in text.Split('\n'))
        {
            // Split on the newline alone so a file written with either line ending survives;
            // the '\r' of a CRLF pair would otherwise ride along into the output.
            var content = line.TrimEnd('\r');

            var header = EntryHeader.Match(content);
            if (header.Success) keeping = ErrorLevels.Contains(header.Groups["level"].Value);

            if (keeping) kept.Add(content);
        }

        return string.Join(Environment.NewLine, kept);
    }

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
