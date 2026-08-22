using System.Reflection;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Devices.Frontpanels;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers the pure half of the user LED mapping: how a DCS-BIOS value decides that a LED is
/// lit, what survives loading a file someone else wrote, and whether the catalog each editor
/// row is built from actually reaches distinct hardware LEDs.
/// </summary>
public class LedMappingTests
{
    private static readonly string[] KnownAircraft = { "A-10C", "F/A-18C" };

    private static LedBinding Binding(
        string aircraft = "A-10C",
        LedDeviceFamily device = LedDeviceFamily.FcuEfis,
        string led = "Ap1",
        string control = "MASTER_CAUTION",
        LedCondition op = LedCondition.NotEquals,
        uint value = 0) =>
        new() { Aircraft = aircraft, Device = device, Led = led, Control = control, Op = op, Value = value };

    // ── Conditions ───────────────────────────────────────────────────────────

    [Theory]
    [InlineData(0u, false)]
    [InlineData(1u, true)]
    [InlineData(65535u, true)]
    public void NotEquals_WithTheDefaultThreshold_LightsOnAnyNonZeroValue(uint value, bool expected) =>
        Assert.Equal(expected, Binding(op: LedCondition.NotEquals).IsOn(value));

    [Theory]
    [InlineData(1u, true)]
    [InlineData(2u, false)]
    [InlineData(3u, true)]
    public void NotEquals_ExcludesTheThreshold(uint value, bool expected) =>
        Assert.Equal(expected, Binding(op: LedCondition.NotEquals, value: 2).IsOn(value));

    [Theory]
    [InlineData(1u, false)]
    [InlineData(2u, true)]
    [InlineData(3u, false)]
    public void Equals_LightsOnlyOnTheThreshold(uint value, bool expected) =>
        Assert.Equal(expected, Binding(op: LedCondition.Equals, value: 2).IsOn(value));

    [Theory]
    [InlineData(1u, false)]
    [InlineData(2u, true)]
    [InlineData(3u, true)]
    public void AtLeast_IncludesTheThreshold(uint value, bool expected) =>
        Assert.Equal(expected, Binding(op: LedCondition.AtLeast, value: 2).IsOn(value));

    [Theory]
    [InlineData(1u, true)]
    [InlineData(2u, true)]
    [InlineData(3u, false)]
    public void AtMost_IncludesTheThreshold(uint value, bool expected) =>
        Assert.Equal(expected, Binding(op: LedCondition.AtMost, value: 2).IsOn(value));

    // ── Sanitize ─────────────────────────────────────────────────────────────

    [Fact]
    public void Sanitize_KeepsAUsableBinding()
    {
        var file = new LedMappingFile { Bindings = { Binding() } };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Single(clean.Bindings);
        Assert.Empty(warnings);
    }

    [Fact]
    public void Sanitize_DropsAnAircraftThisBuildDoesNotKnow()
    {
        var file = new LedMappingFile { Bindings = { Binding(aircraft: "Su-27") } };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Empty(clean.Bindings);
        Assert.Contains(warnings, w => w.Contains("Su-27"));
    }

    [Fact]
    public void Sanitize_DropsALedTheDeviceDoesNotHave()
    {
        // Ap1 is an FCU/EFIS LED; the PAP-3 has no such legend.
        var file = new LedMappingFile { Bindings = { Binding(device: LedDeviceFamily.Pap3, led: "Ap1") } };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Empty(clean.Bindings);
        Assert.Contains(warnings, w => w.Contains("Ap1"));
    }

    [Fact]
    public void Sanitize_DropsABindingWithNoControl()
    {
        var file = new LedMappingFile { Bindings = { Binding(control: "   ") } };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Empty(clean.Bindings);
        Assert.Single(warnings);
    }

    [Fact]
    public void Sanitize_KeepsOneUnusableBindingFromCostingTheRest()
    {
        var file = new LedMappingFile
        {
            Bindings =
            {
                Binding(led: "Ap1", control: "MASTER_CAUTION"),
                Binding(aircraft: "Su-27"),
                Binding(led: "Ap2", control: "GUN_READY"),
            },
        };

        var (clean, _) = file.Sanitize(KnownAircraft);

        Assert.Equal(2, clean.Bindings.Count);
    }

    [Fact]
    public void Sanitize_CollapsesDuplicateSlotsKeepingTheLast()
    {
        var file = new LedMappingFile
        {
            Bindings =
            {
                Binding(control: "FIRST"),
                Binding(control: "SECOND"),
            },
        };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Equal("SECOND", Assert.Single(clean.Bindings).Control);
        Assert.Contains(warnings, w => w.Contains("Duplicate"));
    }

    [Fact]
    public void Sanitize_MatchesAircraftAndLedCaseInsensitively()
    {
        var file = new LedMappingFile { Bindings = { Binding(aircraft: "a-10c", led: "AP1") } };

        var (clean, warnings) = file.Sanitize(KnownAircraft);

        Assert.Single(clean.Bindings);
        Assert.Empty(warnings);
    }

    [Fact]
    public void ForAircraft_ReturnsOnlyThatAircraftsBindings()
    {
        var file = new LedMappingFile
        {
            Bindings = { Binding(aircraft: "A-10C"), Binding(aircraft: "F/A-18C", led: "Ap2") },
        };

        Assert.Equal("Ap2", Assert.Single(file.ForAircraft("F/A-18C")).Led);
    }

    // ── Storage ──────────────────────────────────────────────────────────────

    [Fact]
    public void MissingFile_LoadsAsAnEmptyMapping()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ledmappings-{Guid.NewGuid():N}.json");

        var result = LedMappingStorage.TryLoad(path);

        Assert.True(result.IsSuccess);
        Assert.Empty(result.Value!.Bindings);
    }

    [Fact]
    public void SavedMapping_RoundTrips()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ledmappings-{Guid.NewGuid():N}.json");
        var saved = new LedMappingFile
        {
            Bindings = { Binding(device: LedDeviceFamily.Pap3, led: "CmdA", op: LedCondition.Equals, value: 3) },
        };

        try
        {
            Assert.True(LedMappingStorage.TrySave(saved, path).IsSuccess);

            // Enums are written as names so the file stays legible to whoever it is shared with.
            Assert.Contains("\"Pap3\"", File.ReadAllText(path));

            var loaded = LedMappingStorage.TryLoad(path).Value!;
            var binding = Assert.Single(loaded.Bindings);

            Assert.Equal(LedDeviceFamily.Pap3, binding.Device);
            Assert.Equal("CmdA", binding.Led);
            Assert.Equal(LedCondition.Equals, binding.Op);
            Assert.Equal(3u, binding.Value);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void LegacyNotZeroOperator_LoadsAsNotEquals()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ledmappings-{Guid.NewGuid():N}.json");
        File.WriteAllText(path, """
            {
              "Version": 1,
              "Bindings": [
                {
                  "Aircraft": "A-10C",
                  "Device": "Pap3",
                  "Led": "CmdA",
                  "Control": "MASTER_CAUTION",
                  "Op": "NotZero",
                  "Value": 0
                }
              ]
            }
            """);

        try
        {
            var loaded = LedMappingStorage.TryLoad(path);

            Assert.True(loaded.IsSuccess);
            var binding = Assert.Single(loaded.Value!.Bindings);
            Assert.Equal(LedCondition.NotEquals, binding.Op);
            Assert.True(binding.IsOn(1));
            Assert.False(binding.IsOn(0));
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void UnreadableFile_FailsWithoutThrowing()
    {
        var path = Path.Combine(Path.GetTempPath(), $"ledmappings-{Guid.NewGuid():N}.json");
        File.WriteAllText(path, "{ not json");

        try
        {
            Assert.False(LedMappingStorage.TryLoad(path).IsSuccess);
        }
        finally
        {
            File.Delete(path);
        }
    }

    // ── Catalog ──────────────────────────────────────────────────────────────

    [Fact]
    public void EveryFamilyHasLeds()
    {
        Assert.NotEmpty(LedCatalog.Families);
        Assert.All(LedCatalog.Families, f => Assert.NotEmpty(LedCatalog.For(f)));
    }

    [Fact]
    public void LedIdsAreUniqueWithinAFamily()
    {
        foreach (var family in LedCatalog.Families)
        {
            var ids = LedCatalog.For(family).Select(d => d.Id).ToList();
            Assert.Equal(ids.Count, ids.Distinct(StringComparer.OrdinalIgnoreCase).Count());
        }
    }

    [Fact]
    public void FrontpanelLedsHaveASetter_AndMcduLedsDoNot()
    {
        foreach (var family in LedCatalog.Families.Where(f => f != LedDeviceFamily.Mcdu))
            Assert.All(LedCatalog.For(family), d => Assert.NotNull(d.Set));

        Assert.All(LedCatalog.For(LedDeviceFamily.Mcdu), d => Assert.Null(d.Set));
    }

    [Fact]
    public void EveryMcduLedIdResolvesToAnAnnunciator()
    {
        Assert.All(LedCatalog.For(LedDeviceFamily.Mcdu), d => Assert.NotNull(LedCatalog.ParseMcduLed(d.Id)));
    }

    [Fact]
    public void FcuEfisSettersEachReachTheirOwnLed() =>
        AssertSettersAreDistinct(LedDeviceFamily.FcuEfis, () => new FcuEfisLeds(), LitBoolProperties);

    [Fact]
    public void Pap3SettersEachReachTheirOwnLed() =>
        AssertSettersAreDistinct(LedDeviceFamily.Pap3, () => new Pap3Leds(), LitBoolProperties);

    [Fact]
    public void Agp32SettersEachReachTheirOwnLed() =>
        AssertSettersAreDistinct(
            LedDeviceFamily.Agp32,
            () => new Agp32State.Agp32Leds(),
            leds => ((Agp32State.Agp32Leds)leds).States.Where(s => s.Value).Select(s => s.Key.ToString()));

    /// <summary>
    /// A catalog entry is only useful if it lights exactly one LED, and a different one from
    /// every other entry — a copy-paste slip in a table this long is otherwise invisible until
    /// someone with the hardware notices two legends moving together.
    /// </summary>
    private static void AssertSettersAreDistinct(
        LedDeviceFamily family,
        Func<IFrontpanelLeds> create,
        Func<IFrontpanelLeds, IEnumerable<string>> lit)
    {
        var reached = new Dictionary<string, string>();

        foreach (var descriptor in LedCatalog.For(family))
        {
            var leds = create();
            descriptor.Set!(leds, true);

            var on = lit(leds).ToList();
            var single = Assert.Single(on);

            Assert.False(reached.ContainsKey(single),
                $"{family}: {descriptor.Id} and {reached.GetValueOrDefault(single)} both drive {single}");
            reached[single] = descriptor.Id;
        }
    }

    private static IEnumerable<string> LitBoolProperties(IFrontpanelLeds leds) =>
        leds.GetType()
            .GetProperties(BindingFlags.Public | BindingFlags.Instance)
            .Where(p => p.PropertyType == typeof(bool) && (bool)p.GetValue(leds)!)
            .Select(p => p.Name);
}
