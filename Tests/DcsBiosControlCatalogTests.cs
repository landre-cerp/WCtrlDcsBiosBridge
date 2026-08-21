using DCS_BIOS.ControlLocator;
using WCtrlDcsBiosBridge.UI;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Covers the list the LED editor offers: which of a module's controls can drive a LED, and
/// which of those the picker shows. The JSON fixture is shaped like a DCS-BIOS doc file, so
/// this also pins down that the reader gives us the fields the filter depends on —
/// control_type, the integer output and its max_value.
/// </summary>
public class DcsBiosControlCatalogTests
{
    private const string ModuleJson = """
    {
      "Lights": {
        "MASTER_CAUTION_LT": {
          "category": "Lights",
          "control_type": "led",
          "description": "Master Caution Light",
          "identifier": "MASTER_CAUTION_LT",
          "inputs": [],
          "outputs": [
            { "address": 100, "description": "Master Caution", "mask": 1, "max_value": 1, "shift_by": 0, "suffix": "", "type": "integer" }
          ]
        }
      },
      "Gear": {
        "GEAR_LEVER": {
          "category": "Gear",
          "control_type": "3_position_switch",
          "description": "Gear Lever",
          "identifier": "GEAR_LEVER",
          "inputs": [],
          "outputs": [
            { "address": 104, "description": "Gear Lever", "mask": 3, "max_value": 2, "shift_by": 0, "suffix": "", "type": "integer" }
          ]
        }
      },
      "CDU": {
        "CDU_LINE0": {
          "category": "CDU",
          "control_type": "display",
          "description": "CDU Line 0",
          "identifier": "CDU_LINE0",
          "inputs": [],
          "outputs": [
            { "address": 200, "description": "CDU Line 0", "max_length": 24, "suffix": "", "type": "string" }
          ]
        }
      }
    }
    """;

    /// <summary>Writes the fixture into a directory the control locator can read.</summary>
    private static string WriteModule(string fileName)
    {
        var dir = Path.Combine(Path.GetTempPath(), $"dcsbios-json-{Guid.NewGuid():N}");
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, fileName), ModuleJson);
        return dir;
    }

    [Fact]
    public void ReadsAModuleJsonIntoTheBindableList()
    {
        var fileName = $"TestModule-{Guid.NewGuid():N}.json";
        var dir = WriteModule(fileName);

        try
        {
            DCSBIOSControlLocator.JSONDirectory = dir;
            var controls = DCSBIOSControlLocator.GetModuleControlsFromJson(fileName, onlyDirectResult: true);

            var bindable = DcsBiosControlCatalog.Bindable(controls);

            // The string-output display is not bindable; the lamp and the switch are.
            Assert.Equal(2, bindable.Count);
            Assert.DoesNotContain(bindable, c => c.Identifier == "CDU_LINE0");

            var lamp = Assert.Single(bindable, c => c.Identifier == "MASTER_CAUTION_LT");
            Assert.True(lamp.IsLamp);
            Assert.False(lamp.NeedsCondition);
            Assert.Equal("Master Caution Light (MASTER_CAUTION_LT)", lamp.Display);

            var lever = Assert.Single(bindable, c => c.Identifier == "GEAR_LEVER");
            Assert.False(lever.IsLamp);
            Assert.True(lever.NeedsCondition);
            Assert.Equal(2, lever.MaxValue);
        }
        finally
        {
            Directory.Delete(dir, recursive: true);
        }
    }

    [Fact]
    public void NullControlList_GivesAnEmptyCatalog() =>
        Assert.Empty(DcsBiosControlCatalog.Bindable(null));

    [Fact]
    public void Visible_ShowsOnlyLampsByDefault()
    {
        var all = new[] { Lamp("A"), Switch("B") };

        var visible = DcsBiosControlCatalog.Visible(all, showEverything: false);

        Assert.Equal("A", Assert.Single(visible).Identifier);
    }

    [Fact]
    public void Visible_ShowsEverythingWhenAsked()
    {
        var all = new[] { Lamp("A"), Switch("B") };

        Assert.Equal(2, DcsBiosControlCatalog.Visible(all, showEverything: true).Count);
    }

    [Fact]
    public void Visible_FallsBackToEverythingWhenTheModuleDeclaresNoLamp()
    {
        // Not every module classifies its indicators as control_type "led"; an empty picker
        // would read as a broken app rather than as a filter doing its job.
        var all = new[] { Switch("A"), Switch("B") };

        Assert.Equal(2, DcsBiosControlCatalog.Visible(all, showEverything: false).Count);
        Assert.False(DcsBiosControlCatalog.HasLamps(all));
    }

    private static ControlChoice Lamp(string id) => new(id, id, "Lights", MaxValue: 1, IsLamp: true);

    private static ControlChoice Switch(string id) => new(id, id, "Systems", MaxValue: 2, IsLamp: false);
}
