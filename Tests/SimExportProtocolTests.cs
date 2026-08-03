using System.Text.RegularExpressions;
using Newtonsoft.Json;
using WCtrlDcsBiosBridge.Services;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// The export script is installed into DCS by hand and the app checks its version for equality,
/// so a script and an app that disagree show nothing at all. Both halves ship from this repo,
/// which is the one place the two can be held against each other.
/// </summary>
public class SimExportProtocolTests
{
    private static string Script =>
        File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Scripts", "wctrl-export.lua"));

    [Fact]
    public void ScriptAndAppAgreeOnTheProtocolVersion()
    {
        var declared = Regex.Match(Script, @"PROTOCOL_VERSION\s*=\s*(\d+)");

        Assert.True(declared.Success, "wctrl-export.lua does not declare a PROTOCOL_VERSION");
        Assert.Equal(SimExportReceiver.SupportedVersion, int.Parse(declared.Groups[1].Value));
    }

    /// <summary>
    /// A CNI page carries no marking of its own, so the seat named on the packet is the only
    /// thing keeping the copilot's off a CDU that was given the pilot's.
    /// </summary>
    [Fact]
    public void CniPacketNamesTheSeatItIsFor()
    {
        var packet = JsonConvert.DeserializeObject<SimExportData>(
            """{"ver":3,"cni":{"seat":"copilot","idx":9,"title":"INDEX","n":27}}""");

        Assert.NotNull(packet?.Cni);
        Assert.Equal("copilot", packet.Cni.Seat);
        Assert.Equal(9, packet.Cni.Idx);
    }

    /// <summary>
    /// The seat names are a contract between the two halves: the script writes them, the
    /// listener matches its own against them, and neither says so to the other at runtime.
    /// </summary>
    [Theory]
    [InlineData("pilot")]
    [InlineData("copilot")]
    public void ScriptWritesTheSeatNamesTheAppMatches(string seat)
    {
        var seats = Regex.Match(Script, @"CNI_SEATS\s*=\s*\{([^}]*)\}");

        Assert.True(seats.Success, "wctrl-export.lua does not declare CNI_SEATS");
        Assert.Contains($"\"{seat}\"", seats.Groups[1].Value);
    }
}
