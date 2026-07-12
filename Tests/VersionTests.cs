using WCtrlDcsBiosBridge.Services;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers the version logic that drives the "update available" banner:
/// <see cref="AppVersionProvider.IsPreRelease"/> and the SemVer comparison used to decide
/// whether a GitHub release is newer than the running build.
/// </summary>
public class VersionTests
{
    [Theory]
    [InlineData("3.7.2-beta", true)]
    [InlineData("v3.7.2-beta", true)]
    [InlineData("3.7.2", false)]
    [InlineData("1.0.0", false)]
    public void IsPreRelease_DetectsDashSuffix(string version, bool expected)
    {
        Assert.Equal(expected, AppVersionProvider.IsPreRelease(version));
    }

    [Theory]
    [InlineData("3.7.3", "3.7.2")]        // patch bump
    [InlineData("3.8.0", "3.7.9")]        // minor bump
    [InlineData("4.0.0", "3.99.99")]      // major bump
    [InlineData("v3.7.2", "3.7.2-beta")]  // release outranks its prerelease
    [InlineData("3.7.2-beta.2", "3.7.2-beta.1")] // numeric prerelease ordering
    public void CompareSemVer_LeftIsNewer(string newer, string older)
    {
        Assert.True(GitHubUpdateService.CompareSemVer(newer, older) > 0);
        Assert.True(GitHubUpdateService.CompareSemVer(older, newer) < 0);
    }

    [Theory]
    [InlineData("3.7.2", "v3.7.2")]       // 'v' prefix is ignored
    [InlineData("3.7.2", "3.7.2+build99")] // build metadata is ignored
    public void CompareSemVer_TreatedAsEqual(string a, string b)
    {
        Assert.Equal(0, GitHubUpdateService.CompareSemVer(a, b));
    }

    [Fact]
    public void CompareSemVer_NewerTagBeatsCurrentBuild()
    {
        // The exact shape used in CheckForUpdatesAsync: latestTag vs currentVersion.
        Assert.True(GitHubUpdateService.CompareSemVer("v3.8.0", "3.7.2") > 0);
    }
}
