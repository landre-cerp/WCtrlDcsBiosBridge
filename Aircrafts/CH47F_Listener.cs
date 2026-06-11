using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using System.Linq;

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


    private const string COPILOT_PAGE = "Copilot";

    // Instance field (not static) so each CH47F_Listener has its own color map
    // This is crucial when 2 CDUs are connected - pilot and copilot must have separate color state
    private readonly string[] _pilotColorMap = Enumerable.Range(0, 14)
        .Select(_ => new string(' ', 24))
        .ToArray();

    private readonly string[] _copilotColorMap = Enumerable.Range(0, 14)
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

    private readonly bool switchWithSeat;

    public CH47F_Listener(ICdu? mcdu, UserOptions options, bool pilot = true, bool switchWithSeat = false) : base(mcdu, SupportedAircrafts.CH47, options)
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

    protected override void RegisterMcduControls()
    {
        // --- LED ---
        Register(_MSTR_CAUTION, v => { mcdu!.Leds.Fail = v != 0; mcdu!.RefreshLeds(); });

        // --- Brightness (display step knob) ---
        if (!options.DisableLightingManagement && mcdu != null)
        {
            Register(_PLT_CDU_BRT, v =>
            {
                if (v == 1) _pilot_cdu_brightness = Math.Min(_pilot_cdu_brightness + BRT_STEP, 100);
                ApplyBrightness();
            });
            Register(_PLT_CDU_DIM, v =>
            {
                if (v == 1) _pilot_cdu_brightness = Math.Max(0, _pilot_cdu_brightness - BRT_STEP);
                ApplyBrightness();
            });
            Register(_CPLT_CDU_BRT, v =>
            {
                if (v == 1) _copilot_cdu_brightness = Math.Min(_copilot_cdu_brightness + BRT_STEP, 100);
                ApplyBrightness();
            });
            Register(_CPLT_CDU_DIM, v =>
            {
                if (v == 1) _copilot_cdu_brightness = Math.Max(0, _copilot_cdu_brightness - BRT_STEP);
                ApplyBrightness();
            });
            Register(_PLT_CDU_BACKLIGHT, v =>
            {
                int bright = (int)v * 100 / 65536;
                _pilot_key_brightness = bright;
                _pilot_led_brightness = bright;
                ApplyBrightness();
            });
            Register(_CPLT_CDU_BACKLIGHT, v =>
            {
                int bright = (int)v * 100 / 65536;
                _copilot_key_brightness = bright;
                _copilot_led_brightness = bright;
                ApplyBrightness();
            });
        }

        // --- Seat-position switching ---
        if (switchWithSeat)
        {
            Register(_SEAT_POSITION, v =>
            {
                seatPosition = (int)v;
                _currentPage = seatPosition == PILOT_SEAT ? DEFAULT_PAGE : COPILOT_PAGE;
            });
        }

        // --- CDU lines: pilot always writes to DEFAULT_PAGE ---
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            int lineIndex = i;
            RegisterString(pilotCduColorLines[lineIndex], s =>
            {
                _pilotColorMap[lineIndex] = NormalizeCduString(s);
            });
            RegisterString(pilotCduLines[lineIndex], s =>
            {
                WriteCduLine(DEFAULT_PAGE, lineIndex, NormalizeCduString(s), _pilotColorMap);
            });
        }

        // --- CDU lines: copilot always writes to COPILOT_PAGE ---
        for (int i = 0; i < MAX_CDU_LINES; i++)
        {
            int lineIndex = i;
            RegisterString(copilotCduColorLines[lineIndex], s =>
            {
                _copilotColorMap[lineIndex] = NormalizeCduString(s);
            });
            RegisterString(copilotCduLines[lineIndex], s =>
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
    }

    private void ApplyBrightness()
    {
        if (options.DisableLightingManagement || mcdu == null) return;
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
        mcdu.RefreshBrightnesses();
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