using WCtrlDcsBiosBridge.Aircrafts;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class C130JRegistryTests
{
    [Fact]
    public void DetectedByItsDcsBiosName()
    {
        // Entry/C_130J_30.lua names the aircraft "C-130J-30"; matching is on prefix.
        Assert.Same(AircraftRegistry.C130J, AircraftRegistry.FindByDcsBiosName("C-130J-30"));
    }

    [Fact]
    public void IsInTheRegistry()
    {
        Assert.Contains(AircraftRegistry.C130J, AircraftRegistry.All);
        Assert.Same(AircraftRegistry.C130J, AircraftRegistry.Find(AircraftRegistry.C130J.ModuleId));
    }

    /// <summary>
    /// The id is synthetic because DCS-BIOS has no C-130J. It still has to borrow a real
    /// module's id and json, or the control locator has nothing to load and the listener
    /// throws on construction.
    /// </summary>
    [Fact]
    public void BorrowsARealDcsBiosModule()
    {
        Assert.NotEqual(AircraftRegistry.C130J.ModuleId,
                        AircraftRegistry.C130J.EffectiveDcsBiosModuleId);
        Assert.Equal(AircraftRegistry.A10C.ModuleId,
                     AircraftRegistry.C130J.EffectiveDcsBiosModuleId);
        Assert.Contains(AircraftRegistry.C130J.JsonFile, AircraftRegistry.ExpectedJsonFiles);
    }

    /// <summary>
    /// Both CNIs are carried, so the bridge has a seat to ask about when more than one CDU is
    /// connected. Without this the seat-selection screen never comes up and every CDU shows
    /// the pilot's.
    /// </summary>
    [Fact]
    public void OffersASeatToChoose()
    {
        Assert.True(AircraftRegistry.C130J.HasSeatSelection);
    }

    [Fact]
    public void DoesNotShadowAnotherAircraft()
    {
        var names = AircraftRegistry.All.SelectMany(d => d.DcsBiosNames).ToList();

        Assert.All(AircraftRegistry.C130J.DcsBiosNames, own =>
            Assert.DoesNotContain(names, other =>
                other != own && own.StartsWith(other, StringComparison.OrdinalIgnoreCase)));
    }
}
