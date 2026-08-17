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
}
