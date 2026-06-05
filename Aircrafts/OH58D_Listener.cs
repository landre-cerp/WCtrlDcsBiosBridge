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


            if (e.Address == _MPD_SEL_1!.Address)
            {
                if  (_MPD_SEL_1.GetUIntValue(e.Data) != 0)
                {
                    _mpd_selected = 1;
                }
            }
            if (e.Address == _MPD_SEL_2!.Address)
            {
                if  (_MPD_SEL_2.GetUIntValue(e.Data) != 0)
                {
                    _mpd_selected = 2;
                }
            }
            if (e.Address == _MPD_SEL_3!.Address)
            {
                if  (_MPD_SEL_3.GetUIntValue(e.Data) != 0)
                {
                    _mpd_selected = 3;
                }
            }
            if (e.Address == _MPD_SEL_4!.Address)
            {
                if  (_MPD_SEL_4.GetUIntValue(e.Data) != 0)
                {
                    _mpd_selected = 4;
                }
            }
            if (e.Address == _MPD_SEL_5!.Address)
            {
                if  (_MPD_SEL_5.GetUIntValue(e.Data) != 0)
                {
                    _mpd_selected = 5;
                }
            }

            UpdateMPDLabelsLine();

            // Track RFI selection indicators (will be rendered when RFI string data updates)
            if (e.Address == _RFI_LINE_1_SELECT_COPILOT!.Address)
            {
                var newValue = _RFI_LINE_1_SELECT_COPILOT.GetUIntValue(e.Data);
                if (_rfi1SelectCopilot != newValue)
                {
                    _rfi1SelectCopilot = newValue;
                    UpdateRFILine(8, _rfi1SelectCopilot, _rfi1Channel, _rfi1Cipher, _rfi1Number, _rfi1Frequency, _rfi1SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_1_SELECT_PILOT!.Address)
            {
                var newValue = _RFI_LINE_1_SELECT_PILOT.GetUIntValue(e.Data);
                if (_rfi1SelectPilot != newValue)
                {
                    _rfi1SelectPilot = newValue;
                    UpdateRFILine(8, _rfi1SelectCopilot, _rfi1Channel, _rfi1Cipher, _rfi1Number, _rfi1Frequency, _rfi1SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_2_SELECT_COPILOT!.Address)
            {
                var newValue = _RFI_LINE_2_SELECT_COPILOT.GetUIntValue(e.Data);
                if (_rfi2SelectCopilot != newValue)
                {
                    _rfi2SelectCopilot = newValue;
                    UpdateRFILine(9, _rfi2SelectCopilot, _rfi2Channel, _rfi2Cipher, _rfi2Number, _rfi2Frequency, _rfi2SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_2_SELECT_PILOT!.Address)
            {
                var newValue = _RFI_LINE_2_SELECT_PILOT.GetUIntValue(e.Data);
                if (_rfi2SelectPilot != newValue)
                {
                    _rfi2SelectPilot = newValue;
                    UpdateRFILine(9, _rfi2SelectCopilot, _rfi2Channel, _rfi2Cipher, _rfi2Number, _rfi2Frequency, _rfi2SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_3_SELECT_COPILOT!.Address)
            {
                var newValue = _RFI_LINE_3_SELECT_COPILOT.GetUIntValue(e.Data);
                if (_rfi3SelectCopilot != newValue)
                {
                    _rfi3SelectCopilot = newValue;
                    UpdateRFILine(10, _rfi3SelectCopilot, _rfi3Channel, _rfi3Cipher, _rfi3Number, _rfi3Frequency, _rfi3SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_3_SELECT_PILOT!.Address)
            {
                var newValue = _RFI_LINE_3_SELECT_PILOT.GetUIntValue(e.Data);
                if (_rfi3SelectPilot != newValue)
                {
                    _rfi3SelectPilot = newValue;
                    UpdateRFILine(10, _rfi3SelectCopilot, _rfi3Channel, _rfi3Cipher, _rfi3Number, _rfi3Frequency, _rfi3SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_4_SELECT_COPILOT!.Address)
            {
                var newValue = _RFI_LINE_4_SELECT_COPILOT.GetUIntValue(e.Data);
                if (_rfi4SelectCopilot != newValue)
                {
                    _rfi4SelectCopilot = newValue;
                    UpdateRFILine(11, _rfi4SelectCopilot, _rfi4Channel, _rfi4Cipher, _rfi4Number, _rfi4Frequency, _rfi4SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_4_SELECT_PILOT!.Address)
            {
                var newValue = _RFI_LINE_4_SELECT_PILOT.GetUIntValue(e.Data);
                if (_rfi4SelectPilot != newValue)
                {
                    _rfi4SelectPilot = newValue;
                    UpdateRFILine(11, _rfi4SelectCopilot, _rfi4Channel, _rfi4Cipher, _rfi4Number, _rfi4Frequency, _rfi4SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_5_SELECT_COPILOT!.Address)
            {
                var newValue = _RFI_LINE_5_SELECT_COPILOT.GetUIntValue(e.Data);
                if (_rfi5SelectCopilot != newValue)
                {
                    _rfi5SelectCopilot = newValue;
                    UpdateRFILine(12, _rfi5SelectCopilot, _rfi5Channel, _rfi5Cipher, _rfi5Number, _rfi5Frequency, _rfi5SelectPilot);
                }
            }
            if (e.Address == _RFI_LINE_5_SELECT_PILOT!.Address)
            {
                var newValue = _RFI_LINE_5_SELECT_PILOT.GetUIntValue(e.Data);
                if (_rfi5SelectPilot != newValue)
                {
                    _rfi5SelectPilot = newValue;
                    UpdateRFILine(12, _rfi5SelectCopilot, _rfi5Channel, _rfi5Cipher, _rfi5Number, _rfi5Frequency, _rfi5SelectPilot);
                }
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

            // MPD displays
            if (UpdateCachedValue(_MPD_NG_DISPLAY_FULL, e, v => _mpdNg = v))
            {
                output.Line(2).White().Write("NG  ").Green().WriteLine(FormatValue(_mpdNg, 5, "---"));
            }
            else if (UpdateCachedValue(_MPD_DISPLAY_L, e, v => _mpdLeft = v) || UpdateCachedValue(_MPD_DISPLAY_R, e, v => _mpdRight = v))
            {
                UpdateMPDLabelsLine();
            }
            else if (UpdateCachedValue(_TGT_DISPLAY, e, v => _tgt = v) || UpdateCachedValue(_TRQ_DISPLAY, e, v => _trq = v))
            {
                output.Line(5).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));
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

            // RFI Line 1
            else if (UpdateCachedValue(_RFI_LINE_1_CHANNEL, e, v => _rfi1Channel = v) ||
                     UpdateCachedValue(_RFI_LINE_1_CIPHER, e, v => _rfi1Cipher = v) ||
                     UpdateCachedValue(_RFI_LINE_1_NUMBER, e, v => _rfi1Number = v) ||
                     UpdateCachedValue(_RFI_LINE_1_FREQUENCY, e, v => _rfi1Frequency = v))
            {
                UpdateRFILine(8, _rfi1SelectCopilot, _rfi1Channel, _rfi1Cipher, _rfi1Number, _rfi1Frequency, _rfi1SelectPilot);
            }

            // RFI Line 2
            else if (UpdateCachedValue(_RFI_LINE_2_CHANNEL, e, v => _rfi2Channel = v) ||
                     UpdateCachedValue(_RFI_LINE_2_CIPHER, e, v => _rfi2Cipher = v) ||
                     UpdateCachedValue(_RFI_LINE_2_NUMBER, e, v => _rfi2Number = v) ||
                     UpdateCachedValue(_RFI_LINE_2_FREQUENCY, e, v => _rfi2Frequency = v))
            {
                UpdateRFILine(9, _rfi2SelectCopilot, _rfi2Channel, _rfi2Cipher, _rfi2Number, _rfi2Frequency, _rfi2SelectPilot);
            }

            // RFI Line 3
            else if (UpdateCachedValue(_RFI_LINE_3_CHANNEL, e, v => _rfi3Channel = v) ||
                     UpdateCachedValue(_RFI_LINE_3_CIPHER, e, v => _rfi3Cipher = v) ||
                     UpdateCachedValue(_RFI_LINE_3_NUMBER, e, v => _rfi3Number = v) ||
                     UpdateCachedValue(_RFI_LINE_3_FREQUENCY, e, v => _rfi3Frequency = v))
            {
                UpdateRFILine(10, _rfi3SelectCopilot, _rfi3Channel, _rfi3Cipher, _rfi3Number, _rfi3Frequency, _rfi3SelectPilot);
            }

            // RFI Line 4
            else if (UpdateCachedValue(_RFI_LINE_4_CHANNEL, e, v => _rfi4Channel = v) ||
                     UpdateCachedValue(_RFI_LINE_4_CIPHER, e, v => _rfi4Cipher = v) ||
                     UpdateCachedValue(_RFI_LINE_4_NUMBER, e, v => _rfi4Number = v) ||
                     UpdateCachedValue(_RFI_LINE_4_FREQUENCY, e, v => _rfi4Frequency = v))
            {
                UpdateRFILine(11, _rfi4SelectCopilot, _rfi4Channel, _rfi4Cipher, _rfi4Number, _rfi4Frequency, _rfi4SelectPilot);
            }

            // RFI Line 5
            else if (UpdateCachedValue(_RFI_LINE_5_CHANNEL, e, v => _rfi5Channel = v) ||
                     UpdateCachedValue(_RFI_LINE_5_CIPHER, e, v => _rfi5Cipher = v) ||
                     UpdateCachedValue(_RFI_LINE_5_NUMBER, e, v => _rfi5Number = v) ||
                     UpdateCachedValue(_RFI_LINE_5_FREQUENCY, e, v => _rfi5Frequency = v))
            {
                UpdateRFILine(12, _rfi5SelectCopilot, _rfi5Channel, _rfi5Cipher, _rfi5Number, _rfi5Frequency, _rfi5SelectPilot);
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
        output.Line(3).White().Write($"{leftLabel}").Green().Write($"{FormatValue(_mpdLeft, 3, "---")} ").Write(FormatValue(_mpdRight, 3, "---")).White().WriteLine($" {rightLabel}");
    }

    private void UpdateRFILine(int line, uint selectCopilot, string channel, string cipher, string number, string frequency, uint selectPilot)
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);
        output.Line(line).Column(11)
            .White().Write(number)
            .Green().Write(GetRFILeftArrow(selectCopilot))
            .Green().Write(cipher)
            .White().Write(channel)
            .Green().Write(" ").Write(frequency)
            .WriteLine(GetRFIRightArrow(selectPilot));
    }

    private void InitializeDisplay()
    {
        if (mcdu == null) return;

        // Render static parts of the display once
        var output = GetCompositor(DEFAULT_PAGE);
        output.Clear()
            .Green()
            .Line(0).Centered("OH-58D KIOWA");

        // Line 1 is blank

        // MPD Section (lines 2-5)
        output.Line(2).White().Write("NG  ").Green().WriteLine(FormatValue(_mpdNg, 5, "---"));
        UpdateMPDLabelsLine();
        // Line 4 is blank
        output.Line(5).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));

        // Line 6 is blank

        // CMWS and RFI headers (line 7)
        output.Line(7).Amber().Write("CMWS").Column(12).WriteLine("RFI");

        // CMWS section (lines 8-9)
        output.Line(8).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}");
        output.Line(9).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");

        // RFI lines (lines 8-12) - right side
        UpdateRFILine(8, _rfi1SelectCopilot, _rfi1Channel, _rfi1Cipher, _rfi1Number, _rfi1Frequency, _rfi1SelectPilot);
        UpdateRFILine(9, _rfi2SelectCopilot, _rfi2Channel, _rfi2Cipher, _rfi2Number, _rfi2Frequency, _rfi2SelectPilot);
        UpdateRFILine(10, _rfi3SelectCopilot, _rfi3Channel, _rfi3Cipher, _rfi3Number, _rfi3Frequency, _rfi3SelectPilot);
        UpdateRFILine(11, _rfi4SelectCopilot, _rfi4Channel, _rfi4Cipher, _rfi4Number, _rfi4Frequency, _rfi4SelectPilot);
        UpdateRFILine(12, _rfi5SelectCopilot, _rfi5Channel, _rfi5Cipher, _rfi5Number, _rfi5Frequency, _rfi5SelectPilot);

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
