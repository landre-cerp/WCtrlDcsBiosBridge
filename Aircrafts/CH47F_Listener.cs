using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using System;
using System.Collections.Generic;
using System.Linq;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class CH47F_Listener : AircraftListener
{
    protected const int MAX_CDU_LINES = 14;

    private const int BRT_STEP = 5;
    // Buffers for CDU lines and colors
    private readonly DCSBIOSOutput?[] pilotCduLines = new DCSBIOSOutput?[MAX_CDU_LINES];
    private readonly DCSBIOSOutput?[] pilotCduColorLines = new DCSBIOSOutput?[MAX_CDU_LINES];

    private readonly DCSBIOSOutput?[] copilotCduLines = new DCSBIOSOutput?[MAX_CDU_LINES];
    private readonly DCSBIOSOutput?[] copilotCduColorLines = new DCSBIOSOutput?[MAX_CDU_LINES];

    private DCSBIOSOutput? _MSTR_CAUTION;
    private DCSBIOSOutput? _PLT_CDU_BACKLIGHT;
    private DCSBIOSOutput? _CPLT_CDU_BACKLIGHT;
    private DCSBIOSOutput? _SEAT_POSITION;

    private DCSBIOSOutput? _PLT_CDU_BRT;
    private DCSBIOSOutput? _CPLT_CDU_BRT;

    private DCSBIOSOutput? _PLT_CDU_DIM;
    private DCSBIOSOutput? _CPLT_CDU_DIM;

    private int _pilot_cdu_brightness = 100;
    private int _copilot_cdu_brightness = 100;

    private int _pilot_key_brightness = 100;
    private int _copilot_key_brightness = 100;

    private int _pilot_led_brightness = 100;
    private int _copilot_led_brightness = 100;


    // Which address maps to which line
    private Dictionary<uint, int>? pilotLineMap;
    private Dictionary<uint, int>? pilotColorLines;

    private Dictionary<uint, int>? copilotLineMap;
    private Dictionary<uint, int>? copilotColorLines;

    // Instance field (not static) so each CH47F_Listener has its own color map
    // This is crucial when 2 CDUs are connected - pilot and copilot must have separate color state
    private readonly string[] ColorMap = Enumerable.Range(0, 14)
        .Select(_ => new string(' ', 24))
        .ToArray();

    protected override string GetAircraftName() => SupportedAircrafts.CH47_Name;

    protected override string GetFontFile() => "resources/ch47f-font-21x31.json";

    const int PILOT_SEAT = 0;
    const int COPILOT_SEAT = 1;

    protected int seatPosition = 0;

    private readonly Dictionary<string, Colour> _Colours = new()
    {
        [" "] = Colour.Black,
        ["g"] = Colour.Green,
        ["p"] = Colour.Magenta,
        ["w"] = Colour.White
    };

    public CH47F_Listener(ICdu? mcdu, UserOptions options, bool pilot = true) : base(mcdu, SupportedAircrafts.CH47, options, FrontpanelHub.CreateEmpty())
    {
        seatPosition = pilot ? PILOT_SEAT : COPILOT_SEAT;

        if (options.Ch47CduSwitchWithSeat)
        {
            AddNewPage("Copilot");
        }
    }

    protected override void RegisterMcduControls() { }
    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        // we need to instantiate both PLT and CPLT CDUs to switch between them
        // even if we are only interested in one of them with 2 CDU connected
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            pilotCduLines[i] = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"PLT_CDU_LINE{i+1}");
            pilotCduColorLines[i] = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"PLT_CDU_LINE{i+1}_COLOR");
            copilotCduLines[i] = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"CPLT_CDU_LINE{i + 1}");
            copilotCduColorLines[i] = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"CPLT_CDU_LINE{i + 1}_COLOR");

        }

        _MSTR_CAUTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_MASTER_CAUTION_LIGHT");
        _PLT_CDU_BACKLIGHT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_INT_LIGHT_CDU");
        _CPLT_CDU_BACKLIGHT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CPLT_INT_LIGHT_CDU");
        _SEAT_POSITION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("SEAT_POSITION");

        _PLT_CDU_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_CDU_BRT");
        _CPLT_CDU_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CPLT_CDU_BRT");
        _PLT_CDU_DIM = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PLT_CDU_DIM");
        _CPLT_CDU_DIM = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CPLT_CDU_DIM");

        pilotLineMap = new Dictionary<uint, int>();
        pilotColorLines = new Dictionary<uint, int>();

        copilotLineMap = new Dictionary<uint, int>();
        copilotColorLines = new Dictionary<uint, int>();

        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            pilotLineMap.Add(pilotCduLines[i]!.Address, i + 1);
            pilotColorLines.Add(pilotCduColorLines[i]!.Address, i + 1);
            copilotLineMap.Add(copilotCduLines[i]!.Address, i + 1);
            copilotColorLines.Add(copilotCduColorLines[i]!.Address, i + 1);
        }

    }

    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        var output = GetCompositor("Default");
        var lineMap = pilotLineMap;
        var colorLines = pilotColorLines;

        if (seatPosition == COPILOT_SEAT) { 

            output = GetCompositor("Copilot");
            lineMap = copilotLineMap;
            colorLines = copilotColorLines;
        }

        try
        {
            string data = e.StringData
                .Replace("»", "→")
                .Replace("«", "←")
                .Replace("¡", "☐")
                .Replace("}", "↓")
                .Replace("{", "↑")
                .Replace("®", "Δ");

            output.White();

            if (colorLines!.TryGetValue(e.Address, out int colorLine))
            {
                ColorMap[colorLine - 1] = data;
            }

            if (lineMap!.TryGetValue(e.Address, out int lineIndex))
            {
                // update line with this fast method 
                var screen = pages[DEFAULT_PAGE];  
                var row = screen.Rows[lineIndex - 1];
                var color = ColorMap[lineIndex - 1];
                for (var cellIdx = 0; cellIdx < row.Cells.Length; ++cellIdx)
                {
                    var cell = row.Cells[cellIdx];
                    cell.Character = cellIdx < data.Length ? data[cellIdx] : ' ';
                    _Colours.TryGetValue(color[cellIdx].ToString(), out Colour value);
                    cell.Colour = value;
                    cell.Small = lineIndex % 2 == 0 && lineIndex != 14;
                }

            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Failed to process DCS-BIOS string data");
        }
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        if (mcdu == null) return;
        
        var refresh = false;
        if (e.Address == _MSTR_CAUTION!.Address)
        {
            mcdu.Leds.Fail = _MSTR_CAUTION.GetUIntValue(e.Data) != 0;
            refresh = true;
        }

        if (e.Address == _PLT_CDU_BRT!.Address && (int)_PLT_CDU_BRT.GetUIntValue(e.Data) == 1)
        {
            _pilot_cdu_brightness = Math.Min(_pilot_cdu_brightness+ BRT_STEP, 100);
            refresh = true;
        }




        if (e.Address == _PLT_CDU_DIM!.Address && (int)_PLT_CDU_DIM.GetUIntValue(e.Data) == 1)
        {
            _pilot_cdu_brightness = Math.Max(0, _pilot_cdu_brightness - BRT_STEP);
            refresh = true;
        }




        if (e.Address == _CPLT_CDU_BRT!.Address && (int)_CPLT_CDU_BRT.GetUIntValue(e.Data) == 1)
        {
            _copilot_cdu_brightness = Math.Min(_copilot_cdu_brightness + BRT_STEP, 100);
            refresh = true;
        }




        if (e.Address == _CPLT_CDU_DIM!.Address && (int)_CPLT_CDU_DIM.GetUIntValue(e.Data) == 1)
        {
            _copilot_cdu_brightness = Math.Max(0, _copilot_cdu_brightness - BRT_STEP);
            refresh = true;
        }
        if (e.Address == _PLT_CDU_BACKLIGHT!.Address)
        {
            int bright = (int)_PLT_CDU_BACKLIGHT.GetUIntValue(e.Data);
            bright = bright * 100 / 65536;
            _pilot_key_brightness = bright;
            _pilot_led_brightness = bright;
            refresh = true;
        }
        if (e.Address == _CPLT_CDU_BACKLIGHT!.Address)
        {
            int bright = (int)_CPLT_CDU_BACKLIGHT.GetUIntValue(e.Data);
            bright = bright * 100 / 65536;
            _copilot_key_brightness = bright;
            _copilot_led_brightness = bright;
            refresh = true;
        }


        if (options.Ch47CduSwitchWithSeat && e.Address == _SEAT_POSITION?.Address)
        {
            seatPosition = (int)_SEAT_POSITION.GetUIntValue(e.Data);
        }

        if (!options.DisableLightingManagement && refresh == true)
        {
            if (seatPosition == PILOT_SEAT)
            {
                mcdu.DisplayBrightnessPercent = _pilot_cdu_brightness;
                mcdu.BacklightBrightnessPercent = _pilot_key_brightness;
                mcdu.LedBrightnessPercent = _pilot_led_brightness;
            }
            else
            {
                mcdu.DisplayBrightnessPercent = _copilot_cdu_brightness;
                mcdu.BacklightBrightnessPercent = _copilot_key_brightness;
                mcdu.LedBrightnessPercent = _copilot_led_brightness;
            }
        }

        if (refresh)
        {
            if (!options.DisableLightingManagement) mcdu.RefreshBrightnesses();
            mcdu.RefreshLeds();
        }
    }
}