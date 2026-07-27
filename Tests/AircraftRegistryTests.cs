using WCtrlDcsBiosBridge.Aircrafts;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers <see cref="AircraftRegistry.FindByDcsBiosName"/>, which turns the
/// <c>MetadataStart/_ACFT_NAME</c> string into the descriptor the bridge runs.
///
/// Registered names nest — "F-14BU" starts with the F-14B's "F-14B" — and picking the
/// wrong one fails silently: the aircraft is detected, a listener starts, and only the
/// display stays wrong. These assertions are the only thing standing between that and a
/// reordered list.
/// </summary>
public class AircraftRegistryTests
{
    [Theory]
    [InlineData("F-14B", "F-14B")]
    [InlineData("F-14BU", "F-14B(U)")]
    [InlineData("F-14A-135-GR", "F-14B")]
    [InlineData("A-10C", "A-10C")]
    [InlineData("A-10C_2", "A-10C")]
    public void FindByDcsBiosName_PicksTheLongestRegisteredMatch(string acftName, string expected)
    {
        var descriptor = AircraftRegistry.FindByDcsBiosName(acftName);

        Assert.NotNull(descriptor);
        Assert.Equal(expected, descriptor!.DisplayName);
    }

    [Theory]
    [InlineData("AH-64D_BLK_II", "AH-64D")]
    [InlineData("FA-18C_hornet", "F/A-18C")]
    public void FindByDcsBiosName_StillMatchesOnPrefix(string acftName, string expected)
    {
        var descriptor = AircraftRegistry.FindByDcsBiosName(acftName);

        Assert.NotNull(descriptor);
        Assert.Equal(expected, descriptor!.DisplayName);
    }

    [Theory]
    [InlineData("")]
    [InlineData("NONE")]
    [InlineData("Su-27")]
    public void FindByDcsBiosName_ReturnsNullForUnknownAircraft(string acftName)
    {
        Assert.Null(AircraftRegistry.FindByDcsBiosName(acftName));
    }

    /// <summary>
    /// The F-14B(U) borrows the F-14B's DCS-BIOS module to load its controls, but must keep
    /// a registry id of its own — <see cref="AircraftRegistry.Find"/> resolves on ModuleId,
    /// so a shared one would make the two indistinguishable to the listener factory.
    /// </summary>
    [Fact]
    public void ModuleIds_AreUniqueAcrossTheRegistry()
    {
        var duplicates = AircraftRegistry.All
            .GroupBy(d => d.ModuleId)
            .Where(g => g.Count() > 1)
            .Select(g => g.Key)
            .ToList();

        Assert.Empty(duplicates);
    }

    [Fact]
    public void F14BU_ResolvesItsOwnListenerAndTheF14BDcsBiosModule()
    {
        var f14bu = AircraftRegistry.Find(AircraftRegistry.F14BU.ModuleId);
        var f14b = AircraftRegistry.Find(AircraftRegistry.F14B.ModuleId);

        Assert.Equal("F-14B(U)", f14bu.DisplayName);
        Assert.Equal("F-14B", f14b.DisplayName);
        Assert.Equal(AircraftRegistry.F14B.ModuleId, f14bu.EffectiveDcsBiosModuleId);
    }
}
