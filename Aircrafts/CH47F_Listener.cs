using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using System.Linq;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class CH47F_Listener : AircraftListener
{
    protected const int MAX_CDU_LINES = 14;

    private const int BRT_STEP = 5;
    // Buffers for CDU lines and colors
    private readonly DCSBIOSOutput?[] pilotCduLines = new DCSBIOSOutput?[MAX_CDU_LINES];
    private readonly DCSBIOSOutput?[] pilotCduColorLines = new DCSBIOSOutput?[MAX_CDU_LINES];

    private readonly DCSBIOSOutput?[] copilotCduLines = new DCSBIOSOutput?[MAX_CDU_LINES];
    private readonly DCSBIOSOutput?[] copilotCduColorLines = new DCSBIOSOutput?[MAX_CDU_LINES];

    private int _pilot_cdu_brightness = 100;
    private int _copilot_cdu_brightness = 100;

    private int _pilot_key_brightness = 100;
    private int _copilot_key_brightness = 100;

    private int _pilot_led_brightness = 100;
    private int _copilot_led_brightness = 100;


    private const string COPILOT_PAGE = "Copilot";

    // Instance field (not static) so each CH47F_Listener has its own color map
    // This is crucial when 2 CDUs are connected - pilot and copilot must have separate color state
    private readonly string[] _pilotColorMap = Enumerable.Range(0, 14)
        .Select(_ => new string(' ', 24))
        .ToArray();

    private readonly string[] _copilotColorMap = Enumerable.Range(0, 14)
        .Select(_ => new string(' ', 24))
        .ToArray();

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

    private readonly bool switchWithSeat;

    public CH47F_Listener(UserOptions options, bool pilot = true, bool switchWithSeat = false) : base(AircraftRegistry.CH47, options)
    {
        this.switchWithSeat = switchWithSeat;
        seatPosition = pilot ? PILOT_SEAT : COPILOT_SEAT;

        // Always create the copilot page so both pilot and copilot CDU data
        // can be written independently regardless of seat-switching mode.
        AddNewPage(COPILOT_PAGE);

        // A dedicated copilot device always shows the copilot CDU.
        if (seatPosition == COPILOT_SEAT)
            _currentPage = COPILOT_PAGE;
    }

    protected override void RegisterCduControls()
    {
        // --- LED ---
        RegisterUInt("PLT_MASTER_CAUTION_LIGHT", v => SetCduLeds(fail: v != 0));

        // --- Brightness (display step knob) ---
        RegisterLight("PLT_CDU_BRT", v =>
        {
            if (v == 1) _pilot_cdu_brightness = Math.Min(_pilot_cdu_brightness + BRT_STEP, 100);
            ApplyBrightness();
        });
        RegisterLight("PLT_CDU_DIM", v =>
        {
            if (v == 1) _pilot_cdu_brightness = Math.Max(0, _pilot_cdu_brightness - BRT_STEP);
            ApplyBrightness();
        });
        RegisterLight("CPLT_CDU_BRT", v =>
        {
            if (v == 1) _copilot_cdu_brightness = Math.Min(_copilot_cdu_brightness + BRT_STEP, 100);
            ApplyBrightness();
        });
        RegisterLight("CPLT_CDU_DIM", v =>
        {
            if (v == 1) _copilot_cdu_brightness = Math.Max(0, _copilot_cdu_brightness - BRT_STEP);
            ApplyBrightness();
        });
        RegisterLight("PLT_INT_LIGHT_CDU", v =>
        {
            int bright = (int)v * 100 / 65536;
            _pilot_key_brightness = bright;
            _pilot_led_brightness = bright;
            ApplyBrightness();
        });
        RegisterLight("CPLT_INT_LIGHT_CDU", v =>
        {
            int bright = (int)v * 100 / 65536;
            _copilot_key_brightness = bright;
            _copilot_led_brightness = bright;
            ApplyBrightness();
        });

        // --- Seat-position switching ---
        if (switchWithSeat)
        {
            RegisterUInt("SEAT_POSITION", v =>
            {
                seatPosition = (int)v;
                _currentPage = seatPosition == PILOT_SEAT ? DEFAULT_PAGE : COPILOT_PAGE;
            });
        }

        // --- CDU lines: pilot always writes to DEFAULT_PAGE ---
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            int lineIndex = i;
            RegisterStr(pilotCduColorLines[lineIndex], s =>
            {
                _pilotColorMap[lineIndex] = NormalizeCduString(s);
            });
            RegisterStr(pilotCduLines[lineIndex], s =>
            {
                WriteCduLine(DEFAULT_PAGE, lineIndex, NormalizeCduString(s), _pilotColorMap);
            });
        }

        // --- CDU lines: copilot always writes to COPILOT_PAGE ---
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            int lineIndex = i;
            RegisterStr(copilotCduColorLines[lineIndex], s =>
            {
                _copilotColorMap[lineIndex] = NormalizeCduString(s);
            });
            RegisterStr(copilotCduLines[lineIndex], s =>
            {
                WriteCduLine(COPILOT_PAGE, lineIndex, NormalizeCduString(s), _copilotColorMap);
            });
        }
    }

    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        // we need to instantiate both PLT and CPLT CDUs to switch between them
        // even if we are only interested in one of them with 2 CDU connected
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            pilotCduLines[i]      = ResolveStr($"PLT_CDU_LINE{i+1}");
            pilotCduColorLines[i] = ResolveStr($"PLT_CDU_LINE{i+1}_COLOR");
            copilotCduLines[i]      = ResolveStr($"CPLT_CDU_LINE{i+1}");
            copilotCduColorLines[i] = ResolveStr($"CPLT_CDU_LINE{i+1}_COLOR");
        }
    }

    private void ApplyBrightness()
    {
        if (seatPosition == PILOT_SEAT)
        {
            SetCduDisplayBrightnessPercent(_pilot_cdu_brightness);
            SetCduBacklightBrightnessPercent(_pilot_key_brightness);
            SetCduLedBrightnessPercent(_pilot_led_brightness);
        }
        else
        {
            SetCduDisplayBrightnessPercent(_copilot_cdu_brightness);
            SetCduBacklightBrightnessPercent(_copilot_key_brightness);
            SetCduLedBrightnessPercent(_copilot_led_brightness);
        }
    }

    private static string NormalizeCduString(string s) =>
        s.Replace("»", "→")
         .Replace("«", "←")
         .Replace("¡", "☐")
         .Replace("}", "↓")
         .Replace("{", "↑")
         .Replace("®", "Δ");

    private void WriteCduLine(string pageName, int lineIndex, string data, string[] colorMap)
    {
        var screen = pages[pageName];
        var row = screen.Rows[lineIndex];
        var color = colorMap[lineIndex];
        for (var cellIdx = 0; cellIdx < row.Cells.Length; ++cellIdx)
        {
            var cell = row.Cells[cellIdx];
            cell.Character = cellIdx < data.Length ? data[cellIdx] : ' ';
            _Colours.TryGetValue(color.Length > cellIdx ? color[cellIdx].ToString() : " ", out Colour value);
            cell.Colour = value;
            cell.Small = (lineIndex + 1) % 2 == 0 && (lineIndex + 1) != 14;
        }
    }
}