using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class OH58D_Listener : AircraftListener
{
    private DCSBIOSOutput? _MPD_NG_DISPLAY_FULL;
    private DCSBIOSOutput? _MPD_DISPLAY_L;
    private DCSBIOSOutput? _MPD_DISPLAY_R;
    private DCSBIOSOutput? _MPD_LABEL_L;
    private DCSBIOSOutput? _MPD_LABEL_R;
    private DCSBIOSOutput? _TGT_DISPLAY;
    private DCSBIOSOutput? _TRQ_DISPLAY;
    private DCSBIOSOutput? _CMWS_LINE_1;
    private DCSBIOSOutput? _CMWS_LINE_2;

    private DCSBIOSOutput? _RFI_BRIGHTNESS;
    private DCSBIOSOutput? _MPD_WARN;

    private string _mpdNg = "---";
    private string _mpdLeftLabel = "LEFT";
    private string _mpdRightLabel = "RIGHT";
    private string _mpdLeft = "---";
    private string _mpdRight = "---";
    private string _tgt = "---";
    private string _trq = "---";
    private string _cmwsLine1 = "----";
    private string _cmwsLine2 = "----";

    protected override string GetFontFile() => "resources/a10c-font-21x31.json";
    protected override string GetAircraftName() => SupportedAircrafts.OH58D_Name;

    public OH58D_Listener(ICdu? mcdu, UserOptions options)
        : base(mcdu, SupportedAircrafts.OH58D, options, FrontpanelHub.CreateEmpty())
    {
    }

    protected override void InitializeDcsBiosControls()
    {
        _MPD_NG_DISPLAY_FULL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_NG_DISPLAY_FULL");
        _MPD_DISPLAY_L = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_DISPLAY_L");
        _MPD_DISPLAY_R = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_DISPLAY_R");
        _MPD_LABEL_L = GetFirstAvailableStringOutput("MPD_LABEL_L", "MPD_PARAM_LABEL_L", "MPD_DISPLAY_LABEL_L", "MPD_LEFT_PARAM_LABEL");
        _MPD_LABEL_R = GetFirstAvailableStringOutput("MPD_LABEL_R", "MPD_PARAM_LABEL_R", "MPD_DISPLAY_LABEL_R", "MPD_RIGHT_PARAM_LABEL");
        _TGT_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TGT_DISPLAY");
        _TRQ_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TRQ_DISPLAY");
        _CMWS_LINE_1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_1");
        _CMWS_LINE_2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_2");

        _RFI_BRIGHTNESS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_BRIGHTNESS");
        _MPD_WARN = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_WARN");
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);

            if (mcdu == null) return;

            if (!options.DisableLightingManagement && _RFI_BRIGHTNESS != null && e.Address == _RFI_BRIGHTNESS.Address)
            {
                var brightness = (int)(_RFI_BRIGHTNESS.GetUIntValue(e.Data) * 100 / _RFI_BRIGHTNESS.MaxValue);
                mcdu.BacklightBrightnessPercent = brightness;
                mcdu.DisplayBrightnessPercent = brightness;
                mcdu.LedBrightnessPercent = brightness;
                mcdu.RefreshBrightnesses();
            }

            if (_MPD_WARN != null && e.Address == _MPD_WARN.Address)
            {
                mcdu.Leds.Fail = _MPD_WARN.GetUIntValue(e.Data) == 1;
                mcdu.RefreshLeds();
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS data");
        }
    }

    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        if (mcdu == null) return;

        try
        {
            var hasChanges = false;

            hasChanges |= UpdateCachedValue(_MPD_NG_DISPLAY_FULL, e, v => _mpdNg = v);
            hasChanges |= UpdateCachedValue(_MPD_LABEL_L, e, v => _mpdLeftLabel = v);
            hasChanges |= UpdateCachedValue(_MPD_LABEL_R, e, v => _mpdRightLabel = v);
            hasChanges |= UpdateCachedValue(_MPD_DISPLAY_L, e, v => _mpdLeft = v);
            hasChanges |= UpdateCachedValue(_MPD_DISPLAY_R, e, v => _mpdRight = v);
            hasChanges |= UpdateCachedValue(_TGT_DISPLAY, e, v => _tgt = v);
            hasChanges |= UpdateCachedValue(_TRQ_DISPLAY, e, v => _trq = v);
            hasChanges |= UpdateCachedValue(_CMWS_LINE_1, e, v => _cmwsLine1 = v);
            hasChanges |= UpdateCachedValue(_CMWS_LINE_2, e, v => _cmwsLine2 = v);

            if (!hasChanges) return;

            RenderDisplay(GetCompositor(DEFAULT_PAGE));
            mcdu.RefreshDisplay();
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS string data");
        }
    }

    private static bool UpdateCachedValue(DCSBIOSOutput? output, DCSBIOSStringDataEventArgs e, Action<string> setter)
    {
        if (output == null || e.Address != output.Address) return false;
        setter(e.StringData);
        return true;
    }

    private void RenderDisplay(Compositor output)
    {
        output.Clear()
            .Green()
            .Line(0).Centered("OH-58D KIOWA")
            .Line(2).WriteLine($"NG  {FormatValue(_mpdNg, 5, "---")}")
            .Line(3).WriteLine($"{FormatLabel(_mpdLeftLabel, 7, "LEFT")} {FormatValue(_mpdLeft, 3, "---")}  {FormatLabel(_mpdRightLabel, 7, "RIGHT")} {FormatValue(_mpdRight, 3, "---")}")
            .Line(5).WriteLine($"TGT {FormatValue(_tgt, 3, "---")}   TRQ {FormatValue(_trq, 3, "---")}")
            .Line(10).Amber().WriteLine("CMWS")
            .Line(11).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}")
            .Line(12).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");
    }

    private static DCSBIOSOutput? GetFirstAvailableStringOutput(params string[] outputNames)
    {
        foreach (var outputName in outputNames)
        {
            var output = DCSBIOSControlLocator.GetStringDCSBIOSOutput(outputName);
            if (output != null) return output;
        }

        return null;
    }

    private static string FormatValue(string value, int width, string fallback)
    {
        var normalized = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return normalized.Length <= width ? normalized.PadLeft(width) : normalized[..width];
    }

    private static string FormatLabel(string value, int width, string fallback)
    {
        var normalized = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return normalized.Length <= width ? normalized.PadRight(width) : normalized[..width];
    }
}
