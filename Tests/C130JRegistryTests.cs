using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Devices.Frontpanels;
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
    /// DCS-BIOS carries the aircraft now, so the descriptor names its own module rather than
    /// borrowing the A-10C's. It borrowed one for as long as there was nothing else for the
    /// control locator to load, at the cost of resolving another module's controls.
    /// </summary>
    [Fact]
    public void NamesItsOwnDcsBiosModule()
    {
        Assert.Null(AircraftRegistry.C130J.DcsBiosModuleId);
        Assert.Equal(AircraftRegistry.C130J.ModuleId,
                     AircraftRegistry.C130J.EffectiveDcsBiosModuleId);
        Assert.Equal("C-130J.json", AircraftRegistry.C130J.JsonFile);
        Assert.Contains(AircraftRegistry.C130J.JsonFile, AircraftRegistry.ExpectedJsonFiles);
    }

    /// <summary>
    /// The gear, the master caution and, since DCS-BIOS v0.11.9, the EXEC annunciator are named
    /// controls. EXEC feeds both lamps: EXEC on the PFPs, RDY on the MCDU.
    /// </summary>
    [Fact]
    public void LightsFromNamedControls()
    {
        var defaults = LedDefaults.For(AircraftRegistry.C130J);

        Assert.NotEmpty(defaults.Signals);
        Assert.Contains(defaults.McduLeds, l => l.Led == McduLed.Fail);
        Assert.Contains(defaults.McduLeds, l => l.Led == McduLed.Exec && l.Control == "PLT_CNI_EXEC_LED");
        Assert.Contains(defaults.McduLeds, l => l.Led == McduLed.Rdy && l.Control == "PLT_CNI_EXEC_LED");
        Assert.DoesNotContain(McduLed.Exec, defaults.ComputedMcduLeds.Keys);
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
