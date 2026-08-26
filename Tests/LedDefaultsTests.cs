using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Devices.Frontpanels;
using WCtrlDcsBiosBridge.Devices.Frontpanels.Renderers;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers the built-in LED behaviour now that it is declared rather than written out in each
/// listener: that every declaration reaches a real LED, that the panel applies it, and that
/// the editor can describe it for an aircraft nobody is flying.
/// </summary>
public class LedDefaultsTests
{
    /// <summary>Reaches the renderer's protected members the way a real renderer does.</summary>
    private sealed class TestRenderer : FrontpanelRenderer
    {
        public TestRenderer() : base(new List<IFrontpanelAdapter>(), manageLighting: false) { }

        public override void RenderDisplay(FlightDeckState model) { }

        public override void RenderLeds(FlightDeckState model) { }

        public ulong Build(FlightDeckState model, IFrontpanelLeds leds, LedDeviceFamily family) =>
            BuildLeds(model, leds, family);

        public bool Changed(ulong mask) => LedsChanged(mask);
    }

    private static IEnumerable<AircraftDescriptor> Aircraft => AircraftRegistry.All;

    private static List<string> LitAgp32(Agp32State.Agp32Leds leds) =>
        leds.States.Where(s => s.Value).Select(s => s.Key.ToString()).OrderBy(s => s).ToList();

    // ── The declarations themselves ──────────────────────────────────────────

    [Fact]
    public void EveryDeclaredSignalIsShownBySomeLed()
    {
        var shown = LedCatalog.Families
            .SelectMany(f => LedCatalog.For(f))
            .Select(d => d.Signal)
            .OfType<FlightDeckSignal>()
            .ToHashSet();

        foreach (var descriptor in Aircraft)
        {
            var defaults = LedDefaults.For(descriptor);
            foreach (var signal in defaults.Signals)
            {
                Assert.True(shown.Contains(signal.Signal),
                    $"{descriptor.DisplayName} declares {signal.Signal}, which no LED shows");
            }
        }
    }

    [Fact]
    public void NoAircraftDeclaresTheSameSignalTwice()
    {
        foreach (var descriptor in Aircraft)
        {
            var defaults = LedDefaults.For(descriptor);
            var signals = defaults.Signals.Select(s => s.Signal).ToList();

            Assert.Equal(signals.Count, signals.Distinct().Count());

            // A signal is either read off one control or worked out in code, never both:
            // the editor would have to choose which of the two to report.
            Assert.Empty(signals.Intersect(defaults.ComputedSignals.Keys));

            var leds = defaults.McduLeds.Select(l => l.Led).ToList();
            Assert.Equal(leds.Count, leds.Distinct().Count());
            Assert.Empty(leds.Intersect(defaults.ComputedMcduLeds.Keys));
        }
    }

    [Fact]
    public void EveryDeclaredControlIsNamed()
    {
        foreach (var descriptor in Aircraft)
        {
            var defaults = LedDefaults.For(descriptor);
            Assert.All(defaults.Signals, s =>
            {
                Assert.NotEmpty(s.Controls);
                Assert.All(s.Controls, c => Assert.False(string.IsNullOrWhiteSpace(c)));
            });
            Assert.All(defaults.McduLeds, l => Assert.False(string.IsNullOrWhiteSpace(l.Control)));
        }
    }

    [Fact]
    public void ADirectDefaultReportsItsControl()
    {
        var info = LedDefaults.ForDisplayName("A-10C").For(FlightDeckSignal.GearLeftDown);

        Assert.Equal("GEAR_L_SAFE", Assert.Single(info.Controls!));
        Assert.Equal("GEAR_L_SAFE", info.Describe());
        Assert.Null(info.ComputedFrom);
        Assert.True(info.Exists);
    }

    [Fact]
    public void ASignalCanBeDeclaredFromSeveralControls()
    {
        // The F-14's gear warning lights when any of three OFF lights does.
        var info = LedDefaults.ForDisplayName("F-14B").For(FlightDeckSignal.GearWarning);

        Assert.Equal(3, info.Controls!.Count);
        Assert.Contains("PLT_GEAR_L_OFF_L", info.Controls);
        Assert.Null(info.ComputedFrom);

        // The placeholder has to stay short, so it names one and counts the rest.
        Assert.Equal("PLT_GEAR_L_OFF_L +2", info.Describe());
    }

    [Fact]
    public void AComputedDefaultReportsWhatItIsWorkedOutFrom()
    {
        // The M-2000C reads its lamps out of a raw lighting register: there is no control id
        // to name, which is what makes a written description honest there and nowhere else.
        var info = LedDefaults.ForDisplayName("M-2000C").For(FlightDeckSignal.GearWarning);

        Assert.Null(info.Controls);
        Assert.NotNull(info.ComputedFrom);
        Assert.True(info.Exists);
    }

    [Fact]
    public void AnIdleLedReportsNothing()
    {
        // Nothing drives the A-10C's brake-fan lights, so that row starts empty.
        Assert.False(LedDefaults.ForDisplayName("A-10C").For(FlightDeckSignal.BrkFanOn).Exists);
    }

    [Fact]
    public void AnUnknownAircraftHasNoDefaults() =>
        Assert.Empty(LedDefaults.ForDisplayName("Su-27").Signals);

    [Fact]
    public void McduDefaultsAreDeclaredForTheAircraftThatHadThem()
    {
        Assert.Equal("MASTER_CAUTION", LedDefaults.ForDisplayName("A-10C").For(McduLed.Fail).Describe());
        Assert.Equal("LIGHT_MASTER_CAUTION", LedDefaults.ForDisplayName("F-16C").For(McduLed.Fail).Describe());
        Assert.Equal("MASTER_CAUTION_LT", LedDefaults.ForDisplayName("F/A-18C").For(McduLed.Fail).Describe());
        Assert.Equal("MASTER_CAUTION_IND", LedDefaults.ForDisplayName("UH-1H").For(McduLed.Fail).Describe());

        // The M-2000C reads its annunciators out of a register, so it declares them computed.
        Assert.NotNull(LedDefaults.ForDisplayName("M-2000C").For(McduLed.Rdy).ComputedFrom);
    }

    [Fact]
    public void TheC130JDeclaresItsExecAnnunciatorComputed()
    {
        // No DCS-BIOS module, so there is no control to name: the listener works the lamp out
        // from the CNI page title. Declared on both, because the panels disagree on which lamp
        // they have — EXEC on the PFPs, RDY on the MCDU.
        foreach (var led in new[] { McduLed.Exec, McduLed.Rdy })
        {
            var info = LedDefaults.ForDisplayName("C-130J").For(led);

            Assert.Null(info.Controls);
            Assert.NotNull(info.ComputedFrom);
            Assert.True(info.Exists);
        }
    }

    [Fact]
    public void EveryCduAnnunciatorIsBindable()
    {
        // The catalog is the binding editor's list and McduLed is what the listener writes;
        // a lamp in one and not the other is bindable to nothing, or drivable by nobody.
        var bindable = LedCatalog.For(LedDeviceFamily.Mcdu)
            .Select(d => LedCatalog.ParseMcduLed(d.Id))
            .ToList();

        Assert.All(bindable, led => Assert.NotNull(led));
        Assert.Equal(Enum.GetValues<McduLed>().OrderBy(l => l), bindable.OfType<McduLed>().OrderBy(l => l));
    }

    // ── Applying them ────────────────────────────────────────────────────────

    [Fact]
    public void OneWarningSignalLightsAllFourWarningLeds()
    {
        var model = new FlightDeckState();
        model.SetSignal(FlightDeckSignal.GearWarning, true);
        var leds = new Agp32State.Agp32Leds();

        new TestRenderer().Build(model, leds, LedDeviceFamily.Agp32);

        Assert.Equal(
            new[] { "Gear1Unlk", "Gear2Unlk", "Gear3Unlk", "GearDownRed" }.OrderBy(s => s),
            LitAgp32(leds));
    }

    [Fact]
    public void EachGearSignalLightsItsOwnLed()
    {
        var model = new FlightDeckState();
        model.SetSignal(FlightDeckSignal.GearLeftDown, true);
        model.SetSignal(FlightDeckSignal.GearRightDown, true);
        var leds = new Agp32State.Agp32Leds();

        new TestRenderer().Build(model, leds, LedDeviceFamily.Agp32);

        Assert.Equal(new[] { "Gear1Down", "Gear3Down" }, LitAgp32(leds));
    }

    [Fact]
    public void ASignalTheAircraftDoesNotPublishLeavesItsLedOff()
    {
        var leds = new Agp32State.Agp32Leds();

        new TestRenderer().Build(new FlightDeckState(), leds, LedDeviceFamily.Agp32);

        Assert.Empty(LitAgp32(leds));
    }

    [Fact]
    public void AUserBindingOverridesTheBuiltInOnThatLedAlone()
    {
        var model = new FlightDeckState();
        model.SetSignal(FlightDeckSignal.GearLeftDown, true);
        model.SetSignal(FlightDeckSignal.GearNoseDown, true);
        model.SetUserLed(LedDeviceFamily.Agp32, "Gear1Down", false);
        model.SetUserLed(LedDeviceFamily.Agp32, "TerrOnNdOn", true);

        var leds = new Agp32State.Agp32Leds();
        new TestRenderer().Build(model, leds, LedDeviceFamily.Agp32);

        // Gear1Down was taken over and forced off; the nose light keeps its built-in behaviour.
        Assert.Equal(new[] { "Gear2Down", "TerrOnNdOn" }, LitAgp32(leds));
    }

    [Fact]
    public void PanelsWithNoBuiltInBehaviourStayDarkUntilBound()
    {
        foreach (var family in new[] { LedDeviceFamily.FcuEfis, LedDeviceFamily.Pap3 })
            Assert.All(LedCatalog.For(family), d => Assert.Null(d.Signal));

        Assert.All(LedCatalog.For(LedDeviceFamily.Agp32), d => Assert.NotNull(d.Signal));
    }

    // ── Only sending what changed ────────────────────────────────────────────

    [Fact]
    public void TheFirstFrameIsAlwaysSent()
    {
        // Nothing lit is still a frame the panel has not been told about.
        Assert.True(new TestRenderer().Changed(0));
    }

    [Fact]
    public void AnUnchangedFrameIsNotResent()
    {
        var renderer = new TestRenderer();

        Assert.True(renderer.Changed(0b0101));
        Assert.False(renderer.Changed(0b0101));
        Assert.False(renderer.Changed(0b0101));
    }

    [Fact]
    public void AChangedFrameIsSent()
    {
        var renderer = new TestRenderer();
        renderer.Changed(0b0101);

        Assert.True(renderer.Changed(0b0111));
        Assert.False(renderer.Changed(0b0111));

        // Including going back to a frame sent earlier — the panel shows the latest one.
        Assert.True(renderer.Changed(0b0101));
    }

    [Fact]
    public void InvalidateForcesTheNextFrameOut()
    {
        var renderer = new TestRenderer();
        renderer.Changed(0b0101);
        Assert.False(renderer.Changed(0b0101));

        // Resetting the devices blanks them behind the renderer's back; without this the
        // panel would stay dark until some lamp happened to move.
        renderer.Invalidate();
        Assert.True(renderer.Changed(0b0101));
    }

    [Fact]
    public void TheMaskFollowsTheLitLeds()
    {
        var model = new FlightDeckState();
        var renderer = new TestRenderer();
        var leds = new Agp32State.Agp32Leds();

        var dark = renderer.Build(model, leds, LedDeviceFamily.Agp32);
        Assert.Equal(0UL, dark);

        model.SetSignal(FlightDeckSignal.GearWarning, true);
        var warning = renderer.Build(model, leds, LedDeviceFamily.Agp32);
        Assert.NotEqual(dark, warning);

        // Same state, same mask — this is what stops the frame being resent thirty times a
        // second.
        Assert.Equal(warning, renderer.Build(model, leds, LedDeviceFamily.Agp32));
    }

    [Fact]
    public void EveryLedOfAFamilyFitsInTheMask()
    {
        // One bit per catalog entry, so a family may not outgrow a ulong.
        Assert.All(LedCatalog.Families, f => Assert.True(LedCatalog.For(f).Count <= 64));
    }

    [Fact]
    public void EverySignalCanBeWrittenAndReadBack()
    {
        foreach (var signal in Enum.GetValues<FlightDeckSignal>())
        {
            var model = new FlightDeckState();
            Assert.Null(model.GetSignal(signal));

            model.SetSignal(signal, true);
            Assert.True(model.GetSignal(signal));

            model.SetSignal(signal, false);
            Assert.False(model.GetSignal(signal));
        }
    }
}
