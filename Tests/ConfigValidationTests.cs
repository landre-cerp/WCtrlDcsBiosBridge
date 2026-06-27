using WCtrlDcsBiosBridge.Config;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Validates <see cref="ConfigManager.TryValidate"/> — the pure rule set that decides whether
/// a saved config.json is usable. No file I/O beyond an existing temp directory for the
/// JSON-location check.
/// </summary>
public class ConfigValidationTests
{
    // A config that points at a real directory and is otherwise default-valid.
    private static DcsBiosConfig ValidConfig(string jsonDir) => new()
    {
        DcsBiosJsonLocation = jsonDir,
        ReceiveFromIpUdp = "127.0.0.1",
        SendToIpUdp = "127.0.0.1",
        ReceivePortUdp = 5010,
        SendPortUdp = 7778,
    };

    [Fact]
    public void DefaultConfig_FailsBecauseJsonLocationIsEmpty()
    {
        var result = ConfigManager.TryValidate(new DcsBiosConfig());

        Assert.False(result.IsSuccess);
        Assert.Contains("DcsBiosJsonLocation", result.Error);
    }

    [Fact]
    public void FullyValidConfig_Passes()
    {
        var dir = Directory.CreateTempSubdirectory("wctrl-cfg-test").FullName;
        try
        {
            var result = ConfigManager.TryValidate(ValidConfig(dir));
            Assert.True(result.IsSuccess, result.Error);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void NonExistentJsonLocation_Fails()
    {
        var missing = Path.Combine(Path.GetTempPath(), "wctrl-does-not-exist-" + Guid.NewGuid());
        var result = ConfigManager.TryValidate(ValidConfig(missing));

        Assert.False(result.IsSuccess);
        Assert.Contains("does not exist", result.Error);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    [InlineData(65536)]
    public void OutOfRangePorts_Fail(int port)
    {
        var dir = Directory.CreateTempSubdirectory("wctrl-cfg-test").FullName;
        try
        {
            var cfg = ValidConfig(dir);
            cfg.ReceivePortUdp = port;
            var result = ConfigManager.TryValidate(cfg);
            Assert.False(result.IsSuccess);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Theory]
    [InlineData("not-an-ip")]
    [InlineData("::1")]            // IPv6 is rejected — the bridge requires IPv4
    [InlineData("256.0.0.1")]
    public void InvalidOrNonIpv4Address_Fails(string ip)
    {
        var dir = Directory.CreateTempSubdirectory("wctrl-cfg-test").FullName;
        try
        {
            var cfg = ValidConfig(dir);
            cfg.SendToIpUdp = ip;
            var result = ConfigManager.TryValidate(cfg);
            Assert.False(result.IsSuccess);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }
}
