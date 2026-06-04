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

    // RFI (Radio Frequency Indicator) - 5 lines
    private DCSBIOSOutput? _RFI_LINE_1_CHANNEL;
    private DCSBIOSOutput? _RFI_LINE_1_CIPHER;
    private DCSBIOSOutput? _RFI_LINE_1_NUMBER;
    private DCSBIOSOutput? _RFI_LINE_1_FREQUENCY;
    private DCSBIOSOutput? _RFI_LINE_1_SELECT_COPILOT;
    private DCSBIOSOutput? _RFI_LINE_1_SELECT_PILOT;

    private DCSBIOSOutput? _RFI_LINE_2_CHANNEL;
    private DCSBIOSOutput? _RFI_LINE_2_CIPHER;
    private DCSBIOSOutput? _RFI_LINE_2_NUMBER;
    private DCSBIOSOutput? _RFI_LINE_2_FREQUENCY;
    private DCSBIOSOutput? _RFI_LINE_2_SELECT_COPILOT;
    private DCSBIOSOutput? _RFI_LINE_2_SELECT_PILOT;

    private DCSBIOSOutput? _RFI_LINE_3_CHANNEL;
    private DCSBIOSOutput? _RFI_LINE_3_CIPHER;
    private DCSBIOSOutput? _RFI_LINE_3_NUMBER;
    private DCSBIOSOutput? _RFI_LINE_3_FREQUENCY;
    private DCSBIOSOutput? _RFI_LINE_3_SELECT_COPILOT;
    private DCSBIOSOutput? _RFI_LINE_3_SELECT_PILOT;

    private DCSBIOSOutput? _RFI_LINE_4_CHANNEL;
    private DCSBIOSOutput? _RFI_LINE_4_CIPHER;
    private DCSBIOSOutput? _RFI_LINE_4_NUMBER;
    private DCSBIOSOutput? _RFI_LINE_4_FREQUENCY;
    private DCSBIOSOutput? _RFI_LINE_4_SELECT_COPILOT;
    private DCSBIOSOutput? _RFI_LINE_4_SELECT_PILOT;

    private DCSBIOSOutput? _RFI_LINE_5_CHANNEL;
    private DCSBIOSOutput? _RFI_LINE_5_CIPHER;
    private DCSBIOSOutput? _RFI_LINE_5_NUMBER;
    private DCSBIOSOutput? _RFI_LINE_5_FREQUENCY;
    private DCSBIOSOutput? _RFI_LINE_5_SELECT_COPILOT;
    private DCSBIOSOutput? _RFI_LINE_5_SELECT_PILOT;

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

    // RFI line data - storing channel, cipher, number, frequency, and selection arrows
    private string _rfi1Channel = "";
    private string _rfi1Cipher = "";
    private string _rfi1Number = "";
    private string _rfi1Frequency = "";
    private uint _rfi1SelectCopilot = 0;
    private uint _rfi1SelectPilot = 0;

    private string _rfi2Channel = "";
    private string _rfi2Cipher = "";
    private string _rfi2Number = "";
    private string _rfi2Frequency = "";
    private uint _rfi2SelectCopilot = 0;
    private uint _rfi2SelectPilot = 0;

    private string _rfi3Channel = "";
    private string _rfi3Cipher = "";
    private string _rfi3Number = "";
    private string _rfi3Frequency = "";
    private uint _rfi3SelectCopilot = 0;
    private uint _rfi3SelectPilot = 0;

    private string _rfi4Channel = "";
    private string _rfi4Cipher = "";
    private string _rfi4Number = "";
    private string _rfi4Frequency = "";
    private uint _rfi4SelectCopilot = 0;
    private uint _rfi4SelectPilot = 0;

    private string _rfi5Channel = "";
    private string _rfi5Cipher = "";
    private string _rfi5Number = "";
    private string _rfi5Frequency = "";
    private uint _rfi5SelectCopilot = 0;
    private uint _rfi5SelectPilot = 0;

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

        // RFI Line 1
        _RFI_LINE_1_CHANNEL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_1_CHANNEL");
        _RFI_LINE_1_CIPHER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_1_CIPHER");
        _RFI_LINE_1_NUMBER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_1_NUMBER");
        _RFI_LINE_1_FREQUENCY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_1_FREQUENCY");
        _RFI_LINE_1_SELECT_COPILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_1_SELECT_COPILOT");
        _RFI_LINE_1_SELECT_PILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_1_SELECT_PILOT");

        // RFI Line 2
        _RFI_LINE_2_CHANNEL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_2_CHANNEL");
        _RFI_LINE_2_CIPHER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_2_CIPHER");
        _RFI_LINE_2_NUMBER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_2_NUMBER");
        _RFI_LINE_2_FREQUENCY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_2_FREQUENCY");
        _RFI_LINE_2_SELECT_COPILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_2_SELECT_COPILOT");
        _RFI_LINE_2_SELECT_PILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_2_SELECT_PILOT");

        // RFI Line 3
        _RFI_LINE_3_CHANNEL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_3_CHANNEL");
        _RFI_LINE_3_CIPHER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_3_CIPHER");
        _RFI_LINE_3_NUMBER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_3_NUMBER");
        _RFI_LINE_3_FREQUENCY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_3_FREQUENCY");
        _RFI_LINE_3_SELECT_COPILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_3_SELECT_COPILOT");
        _RFI_LINE_3_SELECT_PILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_3_SELECT_PILOT");

        // RFI Line 4
        _RFI_LINE_4_CHANNEL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_4_CHANNEL");
        _RFI_LINE_4_CIPHER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_4_CIPHER");
        _RFI_LINE_4_NUMBER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_4_NUMBER");
        _RFI_LINE_4_FREQUENCY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_4_FREQUENCY");
        _RFI_LINE_4_SELECT_COPILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_4_SELECT_COPILOT");
        _RFI_LINE_4_SELECT_PILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_4_SELECT_PILOT");

        // RFI Line 5
        _RFI_LINE_5_CHANNEL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_5_CHANNEL");
        _RFI_LINE_5_CIPHER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_5_CIPHER");
        _RFI_LINE_5_NUMBER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_5_NUMBER");
        _RFI_LINE_5_FREQUENCY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("RFI_LINE_5_FREQUENCY");
        _RFI_LINE_5_SELECT_COPILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_5_SELECT_COPILOT");
        _RFI_LINE_5_SELECT_PILOT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_LINE_5_SELECT_PILOT");

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

            // Track RFI selection indicators
            if (_RFI_LINE_1_SELECT_COPILOT != null && e.Address == _RFI_LINE_1_SELECT_COPILOT.Address)
            {
                _rfi1SelectCopilot = _RFI_LINE_1_SELECT_COPILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_1_SELECT_PILOT != null && e.Address == _RFI_LINE_1_SELECT_PILOT.Address)
            {
                _rfi1SelectPilot = _RFI_LINE_1_SELECT_PILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_2_SELECT_COPILOT != null && e.Address == _RFI_LINE_2_SELECT_COPILOT.Address)
            {
                _rfi2SelectCopilot = _RFI_LINE_2_SELECT_COPILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_2_SELECT_PILOT != null && e.Address == _RFI_LINE_2_SELECT_PILOT.Address)
            {
                _rfi2SelectPilot = _RFI_LINE_2_SELECT_PILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_3_SELECT_COPILOT != null && e.Address == _RFI_LINE_3_SELECT_COPILOT.Address)
            {
                _rfi3SelectCopilot = _RFI_LINE_3_SELECT_COPILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_3_SELECT_PILOT != null && e.Address == _RFI_LINE_3_SELECT_PILOT.Address)
            {
                _rfi3SelectPilot = _RFI_LINE_3_SELECT_PILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_4_SELECT_COPILOT != null && e.Address == _RFI_LINE_4_SELECT_COPILOT.Address)
            {
                _rfi4SelectCopilot = _RFI_LINE_4_SELECT_COPILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_4_SELECT_PILOT != null && e.Address == _RFI_LINE_4_SELECT_PILOT.Address)
            {
                _rfi4SelectPilot = _RFI_LINE_4_SELECT_PILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_5_SELECT_COPILOT != null && e.Address == _RFI_LINE_5_SELECT_COPILOT.Address)
            {
                _rfi5SelectCopilot = _RFI_LINE_5_SELECT_COPILOT.GetUIntValue(e.Data);
                needsRefresh = true;
            }
            if (_RFI_LINE_5_SELECT_PILOT != null && e.Address == _RFI_LINE_5_SELECT_PILOT.Address)
            {
                _rfi5SelectPilot = _RFI_LINE_5_SELECT_PILOT.GetUIntValue(e.Data);
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

            // RFI Line 1
            hasChanges |= UpdateCachedValue(_RFI_LINE_1_CHANNEL, e, v => _rfi1Channel = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_1_CIPHER, e, v => _rfi1Cipher = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_1_NUMBER, e, v => _rfi1Number = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_1_FREQUENCY, e, v => _rfi1Frequency = v);

            // RFI Line 2
            hasChanges |= UpdateCachedValue(_RFI_LINE_2_CHANNEL, e, v => _rfi2Channel = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_2_CIPHER, e, v => _rfi2Cipher = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_2_NUMBER, e, v => _rfi2Number = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_2_FREQUENCY, e, v => _rfi2Frequency = v);

            // RFI Line 3
            hasChanges |= UpdateCachedValue(_RFI_LINE_3_CHANNEL, e, v => _rfi3Channel = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_3_CIPHER, e, v => _rfi3Cipher = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_3_NUMBER, e, v => _rfi3Number = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_3_FREQUENCY, e, v => _rfi3Frequency = v);

            // RFI Line 4
            hasChanges |= UpdateCachedValue(_RFI_LINE_4_CHANNEL, e, v => _rfi4Channel = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_4_CIPHER, e, v => _rfi4Cipher = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_4_NUMBER, e, v => _rfi4Number = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_4_FREQUENCY, e, v => _rfi4Frequency = v);

            // RFI Line 5
            hasChanges |= UpdateCachedValue(_RFI_LINE_5_CHANNEL, e, v => _rfi5Channel = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_5_CIPHER, e, v => _rfi5Cipher = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_5_NUMBER, e, v => _rfi5Number = v);
            hasChanges |= UpdateCachedValue(_RFI_LINE_5_FREQUENCY, e, v => _rfi5Frequency = v);

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
            .Line(5).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));

        // CMWS section (left side, lines 8-9)
        output.Line(7).Amber().WriteLine("CMWS");
        output.Line(8).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}");
        output.Line(9).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");

        // RFI (Radio Frequency Indicator) section - right side
        // Format: <arrow> CHANNEL CIPHER NUMBER FREQUENCY <arrow>
        output.Line(7).Column(12).Amber().WriteLine("RFI");
        output.Line(8).Column(12).Green().Write(GetRFIArrow(_rfi1SelectCopilot)).White().Write(_rfi1Channel).Green().Write(_rfi1Cipher).White().Write(_rfi1Number).Green().Write(" ").Write(_rfi1Frequency).WriteLine(GetRFIArrow(_rfi1SelectPilot));
        output.Line(9).Column(12).Green().Write(GetRFIArrow(_rfi2SelectCopilot)).White().Write(_rfi2Channel).Green().Write(_rfi2Cipher).White().Write(_rfi2Number).Green().Write(" ").Write(_rfi2Frequency).WriteLine(GetRFIArrow(_rfi2SelectPilot));
        output.Line(10).Column(12).Green().Write(GetRFIArrow(_rfi3SelectCopilot)).White().Write(_rfi3Channel).Green().Write(_rfi3Cipher).White().Write(_rfi3Number).Green().Write(" ").Write(_rfi3Frequency).WriteLine(GetRFIArrow(_rfi3SelectPilot));
        output.Line(11).Column(12).Green().Write(GetRFIArrow(_rfi4SelectCopilot)).White().Write(_rfi4Channel).Green().Write(_rfi4Cipher).White().Write(_rfi4Number).Green().Write(" ").Write(_rfi4Frequency).WriteLine(GetRFIArrow(_rfi4SelectPilot));
        output.Line(12).Column(12).Green().Write(GetRFIArrow(_rfi5SelectCopilot)).White().Write(_rfi5Channel).Green().Write(_rfi5Cipher).White().Write(_rfi5Number).Green().Write(" ").Write(_rfi5Frequency).WriteLine(GetRFIArrow(_rfi5SelectPilot));
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

    private static string GetRFIArrow(uint value)
    {
        // Return arrow character if selection is active (value == 1)
        return value == 1 ? ">" : " ";
    }
}
