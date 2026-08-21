using WCtrlDcsBiosBridge.Services;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers <see cref="LogFileAccess.TailLines"/> — the part of the log viewer that decides what
/// the user actually sees. Kept as a free function over a <see cref="TextReader"/> precisely so
/// it can be tested here: there is no UI harness in this suite, and log.txt has no rolling
/// policy, so "keeps the last N, not the first N" is the property that has to hold on a file
/// that grows for the life of the install.
/// </summary>
public class LogFileAccessTests
{
    private static string Tail(string text, int maxLines) =>
        LogFileAccess.TailLines(new StringReader(text), maxLines);

    private static string Lines(params int[] numbers) =>
        string.Join(Environment.NewLine, numbers.Select(n => $"line {n}"));

    [Fact]
    public void FewerLinesThanCap_ReturnsEverything()
    {
        var result = Tail(Lines(1, 2, 3), maxLines: 500);

        Assert.Equal(Lines(1, 2, 3), result);
    }

    [Fact]
    public void ExactlyCap_ReturnsEverything()
    {
        var result = Tail(Lines(1, 2, 3), maxLines: 3);

        Assert.Equal(Lines(1, 2, 3), result);
    }

    [Fact]
    public void MoreLinesThanCap_KeepsTheLastOnes()
    {
        var result = Tail(Lines(1, 2, 3, 4, 5), maxLines: 2);

        Assert.Equal(Lines(4, 5), result);
    }

    [Fact]
    public void EmptyInput_ReturnsEmpty()
    {
        Assert.Equal(string.Empty, Tail(string.Empty, maxLines: 500));
    }

    [Fact]
    public void NonPositiveCap_ReturnsEmpty()
    {
        Assert.Equal(string.Empty, Tail(Lines(1, 2, 3), maxLines: 0));
    }

    /// <summary>
    /// The rule that matters: NLog resolves a relative fileName against the application base
    /// directory, not the working directory. Confirmed by launching the app from a different
    /// working directory and finding the log still written beside the executable.
    /// </summary>
    [Fact]
    public void RelativeName_ResolvesAgainstBaseDirectory_NotWorkingDirectory()
    {
        var resolved = LogFileAccess.ResolveAgainstBase("log.txt", @"C:\Program Files\Bridge\");

        Assert.Equal(@"C:\Program Files\Bridge\log.txt", resolved);
        Assert.NotEqual(Path.GetFullPath("log.txt"), resolved);
    }

    [Fact]
    public void AbsoluteName_IsLeftAlone()
    {
        var resolved = LogFileAccess.ResolveAgainstBase(@"D:\logs\bridge.txt", @"C:\Program Files\Bridge\");

        Assert.Equal(@"D:\logs\bridge.txt", resolved);
    }

    /// <summary>
    /// A stack trace is several physical lines and the viewer must not collapse them — the
    /// exception the log viewer exists to show arrives as a multi-line block.
    /// </summary>
    [Fact]
    public void PreservesLineBreaksWithinTheTail()
    {
        const string log = "2026-08-17 10:00:00.0000|ERROR|Failed to start listener\n" +
                           "   at WCtrlDcsBiosBridge.Services.SimExportReceiver.EnsureStarted()\n" +
                           "   at WCtrlDcsBiosBridge.Devices.Cdu.CduDeviceContext.StartBridge()";

        var result = Tail(log, maxLines: 500);

        Assert.Equal(3, result.Split(Environment.NewLine).Length);
        Assert.StartsWith("2026-08-17", result);
        Assert.EndsWith("StartBridge()", result);
    }

    /// <summary>
    /// A log entry as NLog.config lays it out: longdate, level, message.
    /// </summary>
    private static string Entry(string level, string message, params string[] continuation) =>
        string.Join(Environment.NewLine,
            new[] { $"2026-08-17 10:00:00.0000|{level}|{message}" }.Concat(continuation));

    [Fact]
    public void FilterErrors_KeepsErrorEntries_DropsTheRest()
    {
        var log = string.Join(Environment.NewLine,
            Entry("INFO", "Bridge started"),
            Entry("ERROR", "Failed to start listener"),
            Entry("DEBUG", "Polling"));

        var result = LogFileAccess.FilterErrors(log);

        Assert.Equal(Entry("ERROR", "Failed to start listener"), result);
    }

    /// <summary>
    /// The reason this filters entries and not lines: the stack frames under an ERROR carry no
    /// level of their own, and they are the half of the entry worth reading.
    /// </summary>
    [Fact]
    public void FilterErrors_KeepsTheStackTraceUnderAnError()
    {
        var log = string.Join(Environment.NewLine,
            Entry("INFO", "Bridge started"),
            Entry("ERROR", "Failed to start listener",
                  "   at WCtrlDcsBiosBridge.Services.SimExportReceiver.EnsureStarted()",
                  "   at WCtrlDcsBiosBridge.Devices.Cdu.CduDeviceContext.StartBridge()"),
            Entry("INFO", "Retrying"));

        var result = LogFileAccess.FilterErrors(log);

        Assert.Equal(3, result.Split(Environment.NewLine).Length);
        Assert.EndsWith("StartBridge()", result);
    }

    /// <summary>An INFO entry's own continuation lines must not follow the error out.</summary>
    [Fact]
    public void FilterErrors_StopsAtTheNextEntry()
    {
        var log = string.Join(Environment.NewLine,
            Entry("ERROR", "Boom"),
            Entry("INFO", "Recovered", "   continuation of the info entry"));

        var result = LogFileAccess.FilterErrors(log);

        Assert.Equal(Entry("ERROR", "Boom"), result);
    }

    /// <summary>FATAL is an error by any reading a user would give the button.</summary>
    [Fact]
    public void FilterErrors_KeepsFatal()
    {
        var log = string.Join(Environment.NewLine,
            Entry("WARN", "Device busy"),
            Entry("FATAL", "Unrecoverable"));

        var result = LogFileAccess.FilterErrors(log);

        Assert.Equal(Entry("FATAL", "Unrecoverable"), result);
    }

    [Fact]
    public void FilterErrors_WarnIsNotAnError()
    {
        var log = Entry("WARN", "Device busy");

        Assert.Equal(string.Empty, LogFileAccess.FilterErrors(log));
    }

    /// <summary>
    /// TailLines cuts at a line boundary, so the tail can open partway through an entry whose
    /// level was never read. Those orphan lines are dropped rather than guessed at.
    /// </summary>
    [Fact]
    public void FilterErrors_DropsLinesBeforeTheFirstHeader()
    {
        var log = string.Join(Environment.NewLine,
            "   at SomeFrame.FromAnEntryThatWasCutOff()",
            Entry("ERROR", "Boom"));

        var result = LogFileAccess.FilterErrors(log);

        Assert.Equal(Entry("ERROR", "Boom"), result);
    }

    /// <summary>
    /// The log on disk is CRLF, and the result is rejoined with Environment.NewLine — also CRLF
    /// here. The carriage return of the input must not ride along on top of that, or every line
    /// in the panel picks up a stray one.
    /// </summary>
    [Fact]
    public void FilterErrors_HandlesCrLfWithoutLeavingCarriageReturns()
    {
        var log = "2026-08-17 10:00:00.0000|INFO|started@@" +
                  "2026-08-17 10:00:01.0000|ERROR|Boom@@" +
                  "   at Frame()";

        var result = LogFileAccess.FilterErrors(log.Replace("@@", "\r\n"));

        var lines = result.Split(Environment.NewLine);

        Assert.Equal(2, lines.Length);
        Assert.All(lines, line => Assert.DoesNotContain("\r", line));
        Assert.StartsWith("2026-08-17 10:00:01.0000|ERROR|Boom", result);
    }

    [Fact]
    public void FilterErrors_EmptyInput_ReturnsEmpty()
    {
        Assert.Equal(string.Empty, LogFileAccess.FilterErrors(string.Empty));
    }
}
