using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

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

    private DCSBIOSOutput? _CLOCK_HOURS;
    private DCSBIOSOutput? _CLOCK_MINUTES;
    private DCSBIOSOutput? _CLOCK_SECONDS;

    private string _mpdLeft = "---";
    private string _mpdRight = "---";
    private string _tgt = "---";
    private string _trq = "---";
    private string _cmwsLine1 = "----";
    private string _cmwsLine2 = "----";

    // RFI line data - storing channel, cipher, number, frequency, and selection arrows
    private readonly RFILineData[] _rfiData =
    [
        new RFILineData(),
        new RFILineData(),
        new RFILineData(),
        new RFILineData(),
        new RFILineData()
    ];

    private uint _mpd_selected= 0;

    private static readonly Dictionary<int, (string Left, string Right)> SelectorLabels = new()
    {
        { 1, ("   batt v", "  start v") },    
        { 2, (" rect ld%", "s gen ld%") },
        { 3, ("      acv", "   rect v") },         
        { 4, ("fuelt qty", " eng trq%") }, 
        { 5, ("       nr", "       np") }              
    };
    private const int TGT_LINE=2;
    private const int NG_LINE=3;
    private const int MPD_LINE = 5;
    private const int RFI_START_LINE = 8;

    private string _clockHh = "00";
    private string _clockMm = "00";
    private string _clockSs = "00";


    public OH58D_Listener(ICdu? mcdu, UserOptions options)
        : base(mcdu, AircraftRegistry.OH58D, options)
    {
    }
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
                Cipher = DCSBIOSControlLocator.GetUIntDCSBIOSOutput($"RFI_LINE_{lineNum}_CIPHER"),
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

        _CLOCK_MINUTES = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_MINUTES");
        _CLOCK_SECONDS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_SECONDS");
        _CLOCK_HOURS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_HOURS");

        InitializeDisplay();
    }


    protected override void RegisterCduControls() {

        if (!options.DisableLightingManagement && mcdu != null) {
            Register(_RFI_BRIGHTNESS, v =>
            {
                var brightness = (int)(_RFI_BRIGHTNESS!.GetUIntValue(v) * 100 / _RFI_BRIGHTNESS.MaxValue);
                mcdu.BacklightBrightnessPercent = brightness;
                mcdu.DisplayBrightnessPercent = brightness;
                mcdu.LedBrightnessPercent = brightness;
                mcdu.RefreshBrightnesses();
            });
        }

        Register(_MPD_SEL_1, v => { if (v != 0) { _mpd_selected = 1; } });
        Register(_MPD_SEL_2, v => { if (v != 0) { _mpd_selected = 2; } });
        Register(_MPD_SEL_3, v => { if (v != 0) { _mpd_selected = 3; } });
        Register(_MPD_SEL_4, v => { if (v != 0) { _mpd_selected = 4; } });
        Register(_MPD_SEL_5, v => { if (v != 0) { _mpd_selected = 5; } });

        RegisterString(_CMWS_LINE_1, v =>
        {
            GetCompositor(DEFAULT_PAGE).Line(9).Green().WriteLine($"1 {FormatValue(v, 4, "----")}");
        });
        RegisterString(_CMWS_LINE_2, v => {
            GetCompositor(DEFAULT_PAGE).Line(10).Green().WriteLine($"2 {FormatValue(v, 4, "----")}");
        });
        
        RegisterString(_MPD_NG_DISPLAY_FULL, v =>
        {
            GetCompositor(DEFAULT_PAGE).Line(NG_LINE).White().Write("NG ").Green().WriteLine(FormatValue(v, 5, "---"));
        });


        for (int i = 0; i < 5; i++)
        {
            int idx = i;  // capture loop variable
            Register(_rfiOutputs[idx].SelectCopilot,  v => { _rfiData[idx].SelectCopilot = v;  UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
            Register(_rfiOutputs[idx].SelectPilot,    v => { _rfiData[idx].SelectPilot = v;    UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
            Register(_rfiOutputs[idx].Cipher, v => { _rfiData[idx].Cipher = v==1 ? "☐" : " "; UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
            RegisterString(_rfiOutputs[idx].Channel,  v => { _rfiData[idx].Channel = v;        UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
            
            RegisterString(_rfiOutputs[idx].Number,   v => { _rfiData[idx].Number = v;         UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
            RegisterString(_rfiOutputs[idx].Frequency,v => { _rfiData[idx].Frequency = v;      UpdateRFILine(RFI_START_LINE + idx, _rfiData[idx]); });
        }

        RegisterString(_MPD_DISPLAY_L, v =>
        {
            _mpdLeft = v;
            var (leftLabel, rightLabel) = GetActiveLabels();
            GetCompositor(DEFAULT_PAGE)
                .Line(MPD_LINE).White().Write($"{leftLabel} ").Green().Write($"{FormatValue(_mpdLeft, 3, "---")}");
            
        });

        RegisterString(_MPD_DISPLAY_R, v =>
        {
            _mpdRight = v;
            var (leftLabel, rightLabel) = GetActiveLabels();
            GetCompositor(DEFAULT_PAGE)
                .Line(MPD_LINE + 1).White().Write($"{rightLabel} ").Green().Write($"{FormatValue(_mpdRight, 3, "---")}");
            
        });

        RegisterString(_TGT_DISPLAY, v => { _tgt = v; UpdateTgtTrq(); });
        RegisterString(_TRQ_DISPLAY, v => { _trq = v;UpdateTgtTrq(); } );

        RegisterString(_CLOCK_HOURS, v => { _clockHh = FormatValue(v, 2, "00"); UpdateClock(); });
        RegisterString(_CLOCK_MINUTES, v => { _clockMm = FormatValue(v, 2, "00"); UpdateClock(); });
        RegisterString(_CLOCK_SECONDS, v => { _clockSs = FormatValue(v, 2, "00"); UpdateClock(); });

    }

    private void UpdateClock()
    {
        FlightDeck.Agp32UtcTime = _clockHh+_clockMm+_clockSs;
    }

    protected override void RegisterFrontpanelControls() { }


    private void UpdateTgtTrq()
    {
        var output = GetCompositor(DEFAULT_PAGE);
        output.Line(TGT_LINE).White().Write("TGT ").Green().Write($"{FormatValue(_tgt, 3, "---")}   ").White().Write("TRQ ").Green().WriteLine(FormatValue(_trq, 3, "---"));
    }

    private void UpdateRFILine(int line, RFILineData data)
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);
        output.Line(line).Column(11)
            .White().Write(data.Number)
            .Green().Write(GetRFILeftArrow(data.SelectCopilot))
            .White().Write(data.Channel)
            .Green().Write(data.Cipher)
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

        output.Line(7).Amber().Column(17).WriteLine("RFI");
        output.Line(8).Write("CMWS");
        
        output.Line(9).Green().WriteLine($"1 {FormatValue(_cmwsLine1, 4, "----")}");
        output.Line(10).Green().WriteLine($"2 {FormatValue(_cmwsLine2, 4, "----")}");


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
        return value == 1 ? "←" : " ";
    }

    private static string GetRFIRightArrow(uint value)
    {
        // Return right arrow character if pilot selection is active (value == 1)
        return value == 1 ? "→" : " ";
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
