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
    private readonly RFILineOutputs[] _rfiOutputs = new RFILineOutputs[5];

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
    private readonly RFILineData[] _rfiData = new RFILineData[5]
    {
        new RFILineData(),
        new RFILineData(),
        new RFILineData(),
        new RFILineData(),
        new RFILineData()
    };

    private uint _mpd_selected= 0;

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
    private const int TGT_LINE=2;
    private const int NG_LINE=3;
    private const int MPD_LINE = 5;

    protected override string GetFontFile() => "resources/oh58d-font-21x31.json";
    protected override string GetAircraftName() => SupportedAircrafts.OH58D_Name;

    public OH58D_Listener(ICdu? mcdu, UserOptions options)
        : base(mcdu, SupportedAircrafts.OH58D, options, FrontpanelHub.CreateEmpty())
    {
    }

    protected override void RegisterLightingControls() { }
    protected override void RegisterMcduControls() { }
    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        _MPD_NG_DISPLAY_FULL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_NG_DISPLAY_FULL");
        _MPD_DISPLAY_L = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_DISPLAY_L");
        _MPD_DISPLAY_R = DCSBIOSControlLocator.GetStringDCSBIOSOutput("MPD_DISPLAY_R");
        _TGT_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TGT_DISPLAY");
        _TRQ_DISPLAY = DCSBIOSControlLocator.GetStringDCSBIOSOutput("TRQ_DISPLAY");
        _CMWS_LINE_1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_1");
        _CMWS_LINE_2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMWS_LINE_2");

        // Initialize RFI outputs for all 5 lines
        for (int i = 0; i < 5; i++)
        {
            int lineNum = i + 1;
            _rfiOutputs[i] = new RFILineOutputs
            {
                Channel = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"RFI_LINE_{lineNum}_CHANNEL"),
                Cipher = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"RFI_LINE_{lineNum}_CIPHER"),
                Number = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"RFI_LINE_{lineNum}_NUMBER"),
                Frequency = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"RFI_LINE_{lineNum}_FREQUENCY"),
                SelectCopilot = DCSBIOSControlLocator.GetUIntDCSBIOSOutput($"RFI_LINE_{lineNum}_SELECT_COPILOT"),
                SelectPilot = DCSBIOSControlLocator.GetUIntDCSBIOSOutput($"RFI_LINE_{lineNum}_SELECT_PILOT")
            };
        }

        _MPD_SEL_1 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_1");
        _MPD_SEL_2 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_2");
        _MPD_SEL_3 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_3");
        _MPD_SEL_4 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_4");
        _MPD_SEL_5 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MPD_SEL_5");

        _RFI_BRIGHTNESS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("RFI_BRIGHTNESS");

        InitializeDisplay();
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

            if (e.Address == _MPD_SEL_1!.Address && _MPD_SEL_1.GetUIntValue(e.Data) != 0)
            {
                _mpd_selected = 1;
            }
            if (e.Address == _MPD_SEL_2!.Address && _MPD_SEL_2.GetUIntValue(e.Data) != 0)
            {
                _mpd_selected = 2;
            }
            if (e.Address == _MPD_SEL_3!.Address && _MPD_SEL_3.GetUIntValue(e.Data) != 0)
            {
                _mpd_selected = 3;
            }
            if (e.Address == _MPD_SEL_4!.Address && _MPD_SEL_4.GetUIntValue(e.Data) != 0)
            {
                _mpd_selected = 4;
            }
            if (e.Address == _MPD_SEL_5!.Address && _MPD_SEL_5.GetUIntValue(e.Data) != 0)
            {
                _mpd_selected = 5;
            }

            UpdateMPDLabelsLine();

            // Track RFI selection indicators for all 5 lines
            for (int i = 0; i < 5; i++)
            {
                if (e.Address == _rfiOutputs[i].SelectCopilot!.Address)
                {
                    _rfiData[i].SelectCopilot = _rfiOutputs[i].SelectCopilot.GetUIntValue(e.Data);
                }
                if (e.Address == _rfiOutputs[i].SelectPilot!.Address)
                {
                    _rfiData[i].SelectPilot = _rfiOutputs[i].SelectPilot.GetUIntValue(e.Data);
                }
            }

            // Update all RFI lines on display
            for (int i = 0; i < 5; i++)
            {
                UpdateRFILine(8 + i, _rfiData[i]);
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
            var output = GetCompositor(DEFAULT_PAGE);

            if (UpdateCachedValue(_MPD_NG_DISPLAY_FULL, e, v => _mpdNg = v))
            {
                output.Line(NG_LINE).White().Write("NG ").Green().WriteLine(FormatValue(_mpdNg, 5, "---"));
            }
            else if (UpdateCachedValue(_MPD_DISPLAY_L, e, v => _mpdLeft = v) || UpdateCachedValue(_MPD_DISPLAY_R, e, v => _mpdRight = v))
            {
                UpdateMPDLabelsLine();
            }
            else if (UpdateCachedValue(_TGT_DISPLAY, e, v => _tgt = v) || UpdateCachedValue(_TRQ_DISPLAY, e, v => _trq = v))
            {
                output.Line(TGT_LINE).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));
            }

            // CMWS displays
            else if (UpdateCachedValue(_CMWS_LINE_1, e, v => _cmwsLine1 = v))
            {
                output.Line(8).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}");
            }
            else if (UpdateCachedValue(_CMWS_LINE_2, e, v => _cmwsLine2 = v))
            {
                output.Line(9).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");
            }

            // RFI Lines - process all 5 lines
            for (int i = 0; i < 5; i++)
            {
                if (UpdateCachedValue(_rfiOutputs[i].Channel, e, v => _rfiData[i].Channel = v) ||
                    UpdateCachedValue(_rfiOutputs[i].Cipher, e, v => _rfiData[i].Cipher = v) ||
                    UpdateCachedValue(_rfiOutputs[i].Number, e, v => _rfiData[i].Number = v) ||
                    UpdateCachedValue(_rfiOutputs[i].Frequency, e, v => _rfiData[i].Frequency = v))
                {
                    UpdateRFILine(8 + i, _rfiData[i]);
                    break;
                }
            }
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

    private void UpdateMPDLabelsLine()
    {
        if (mcdu == null) return;
        var (leftLabel, rightLabel) = GetActiveLabels();
        var output = GetCompositor(DEFAULT_PAGE);
        output.Line(MPD_LINE).White().Write($"{leftLabel}").Green().Write($"{FormatValue(_mpdLeft, 3, "---")} ").Write(FormatValue(_mpdRight, 3, "---")).White().WriteLine($" {rightLabel}");
    }

    private void UpdateRFILine(int line, RFILineData data)
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);
        output.Line(line).Column(11)
            .White().Write(data.Number)
            .Green().Write(GetRFILeftArrow(data.SelectCopilot))
            .Green().Write(data.Cipher)
            .White().Write(data.Channel)
            .Green().Write(" ").Write(data.Frequency)
            .WriteLine(GetRFIRightArrow(data.SelectPilot));
    }

    private void InitializeDisplay()
    {
        if (mcdu == null) return;

        // Render static parts of the display once
        var output = GetCompositor(DEFAULT_PAGE);
        output.Clear()
            .Green()
            .Line(0).Centered("OH-58D KIOWA");

        output.Line(TGT_LINE).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));
        
        output.Line(NG_LINE).White().Write("NG ").Green().WriteLine(FormatValue(_mpdNg, 5, "---"));
        UpdateMPDLabelsLine();

        
        output.Line(7).Amber().Write("CMWS").Column(17).WriteLine("RFI");
        
        output.Line(8).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}");
        output.Line(9).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");


        for (int i = 0; i < 5; i++)
        {
            UpdateRFILine(8 + i, _rfiData[i]);
        }

        mcdu.RefreshDisplay();
    }

    private (string Left, string Right) GetActiveLabels()
    {
        if (SelectorLabels.TryGetValue((int)_mpd_selected, out var labels)) return labels;

        // Default labels if no selector is active
        return ("", "");
    }

    private static string FormatValue(string value, int width, string fallback)
    {
        var normalized = string.IsNullOrWhiteSpace(value) ? fallback : value.Trim();
        return normalized.Length <= width ? normalized.PadLeft(width) : normalized[..width];
    }

    private static string GetRFILeftArrow(uint value)
    {
        // Return left arrow character if copilot selection is active (value == 1)
        return value == 1 ? "<" : " ";
    }

    private static string GetRFIRightArrow(uint value)
    {
        // Return right arrow character if pilot selection is active (value == 1)
        return value == 1 ? ">" : " ";
    }
}

// Helper classes to group related data
internal class RFILineOutputs
{
    public DCSBIOSOutput? Channel { get; set; }
    public DCSBIOSOutput? Cipher { get; set; }
    public DCSBIOSOutput? Number { get; set; }
    public DCSBIOSOutput? Frequency { get; set; }
    public DCSBIOSOutput? SelectCopilot { get; set; }
    public DCSBIOSOutput? SelectPilot { get; set; }
}

internal class RFILineData
{
    public string Channel { get; set; } = "";
    public string Cipher { get; set; } = "";
    public string Number { get; set; } = "";
    public string Frequency { get; set; } = "";
    public uint SelectCopilot { get; set; } = 0;
    public uint SelectPilot { get; set; } = 0;
}
