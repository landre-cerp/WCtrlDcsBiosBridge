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
    private DCSBIOSOutput? _TGT_DISPLAY;
    private DCSBIOSOutput? _TRQ_DISPLAY;
    private DCSBIOSOutput? _CMWS_LINE_1;
    private DCSBIOSOutput? _CMWS_LINE_2;

    private DCSBIOSOutput? _MPD_SEL_1;
    private DCSBIOSOutput? _MPD_SEL_2;
    private DCSBIOSOutput? _MPD_SEL_3;
    private DCSBIOSOutput? _MPD_SEL_4;
    private DCSBIOSOutput? _MPD_SEL_5;

    private DCSBIOSOutput? _RFI_BRIGHTNESS;

    private string _mpdNg = "---";
    private string _mpdLeft = "---";
    private string _mpdRight = "---";
    private string _tgt = "---";
    private string _trq = "---";
    private string _cmwsLine1 = "----";
    private string _cmwsLine2 = "----";

    private uint _sel1 = 0;
    private uint _sel2 = 0;
    private uint _sel3 = 0;
    private uint _sel4 = 0;
    private uint _sel5 = 0;

    // Mapping of selector position to (LeftLabel, RightLabel)
    // Based on OH-58D MPD panel layout (from bottom to top)
    private static readonly Dictionary<int, (string Left, string Right)> SelectorLabels = new()
    {
        { 1, ("   batt v", "start v") },    
        { 2, (" rect ld%", "s gen ld%") },
        { 3, ("      acv", "rect v") },         
        { 4, ("fuelt qty", "eng trq%") }, 
        { 5, ("       nr", "np") }              
    };

    protected override string GetFontFile() => "resources/oh58d-font-21x31.json";
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
        _TGT_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TGT_DISPLAY");
        _TRQ_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TRQ_DISPLAY");
        _CMWS_LINE_1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_1");
        _CMWS_LINE_2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_2");

        _MPD_SEL_1 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_1");
        _MPD_SEL_2 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_2");
        _MPD_SEL_3 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_3");
        _MPD_SEL_4 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_4");
        _MPD_SEL_5 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_5");

        _RFI_BRIGHTNESS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_BRIGHTNESS");
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);

            if (mcdu == null) return;

            var needsRefresh = false;

            if (!options.DisableLightingManagement && _RFI_BRIGHTNESS != null && e.Address == _RFI_BRIGHTNESS.Address)
            {
                var brightness = (int)(_RFI_BRIGHTNESS.GetUIntValue(e.Data) * 100 / _RFI_BRIGHTNESS.MaxValue);
                mcdu.BacklightBrightnessPercent = brightness;
                mcdu.DisplayBrightnessPercent = brightness;
                mcdu.LedBrightnessPercent = brightness;
                mcdu.RefreshBrightnesses();
            }

            // Track selector positions
            if (_MPD_SEL_1 != null && e.Address == _MPD_SEL_1.Address)
            {
                _sel1 = _MPD_SEL_1.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_MPD_SEL_2 != null && e.Address == _MPD_SEL_2.Address)
            {
                _sel2 = _MPD_SEL_2.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_MPD_SEL_3 != null && e.Address == _MPD_SEL_3.Address)
            {
                _sel3 = _MPD_SEL_3.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_MPD_SEL_4 != null && e.Address == _MPD_SEL_4.Address)
            {
                _sel4 = _MPD_SEL_4.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_MPD_SEL_5 != null && e.Address == _MPD_SEL_5.Address)
            {
                _sel5 = _MPD_SEL_5.GetUIntValue(e.Data);
                needsRefresh = true;
            }

            if (needsRefresh)
            {
                RenderDisplay(GetCompositor(DEFAULT_PAGE));
                mcdu.RefreshDisplay();
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
        // Determine which selector is active and get corresponding labels
        var (leftLabel, rightLabel) = GetActiveLabels();

        output.Clear()
            .Green()
            .Line(0).Centered("OH-58D KIOWA")
            .Line(2).White().Write("NG  ").Green().WriteLine(FormatValue(_mpdNg, 5, "---"))
            .Line(3).White().Write($"{leftLabel}").Green().Write($"{FormatValue(_mpdLeft, 3, "---")} ").Write(FormatValue(_mpdRight, 3, "---")).White().WriteLine($" {rightLabel}")
            .Line(5).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"))
            .Line(10).Amber().WriteLine("CMWS")
            .Line(11).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}")
            .Line(12).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");
    }

    private (string Left, string Right) GetActiveLabels()
    {
        // Check which selector is active (value == 65535)
        if (_sel1 == 65535 && SelectorLabels.TryGetValue(1, out var labels1)) return labels1;
        if (_sel2 == 65535 && SelectorLabels.TryGetValue(2, out var labels2)) return labels2;
        if (_sel3 == 65535 && SelectorLabels.TryGetValue(3, out var labels3)) return labels3;
        if (_sel4 == 65535 && SelectorLabels.TryGetValue(4, out var labels4)) return labels4;
        if (_sel5 == 65535 && SelectorLabels.TryGetValue(5, out var labels5)) return labels5;

        // Default labels if no selector is active
        return ("", "");
    }

    private static string FormatValue(string value, int width, string fallback)
    {
        var normalized = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return normalized.Length <= width ? normalized.PadLeft(width) : normalized[..width];
    }
}
