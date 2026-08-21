using DCS_BIOS.Serialized;
using WCtrlDcsBiosBridge.Services;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class A10C_Listener : AircraftListener
{
    private const int BRT_STEP = 10;
    private const string TAKEOFF_PAGE = "TAKEOFF";
    private const string LANDING_PAGE = "LANDING";
    private readonly DCSBIOSOutput?[] cduLines = new DCSBIOSOutput?[10];

    private readonly Key _nextPageKey;
    private readonly Key _prevPageKey;
    private readonly Key _perfPageKey;
    private readonly bool _perfPagesEnabled;

    private SimExportReceiver? _exportReceiver;
    private bool _windManuallySet;
    private bool _elevManuallySet;

    // multi-use: registered in both RegisterCduControls and RegisterFrontpanelControls
    private DCSBIOSOutput? _CONSOLE_BRT;

    private int[] pressureDigits = new int[4];
    private int[] altitudeDigits = new int[3];
    private int altitudeFine;

    private bool _clockShowsEt;
    private string _clockHh = "00";
    private string _clockMm = "00";
    private string _clockSs = "00";

    public A10C_Listener(
        UserOptions options) : base(AircraftRegistry.A10C, options) {
        _nextPageKey = Enum.TryParse<Key>(options.A10C.NextPageKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _prevPageKey = Enum.TryParse<Key>(options.A10C.PrevPageKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;
        _perfPageKey = Enum.TryParse<Key>(options.A10C.PerfPageKey, out var perfKey)
            ? perfKey
            : Key.FuelPred;
        _perfPagesEnabled = options.A10C.EnablePerfPages;

        if (_perfPagesEnabled)
        {
            AddNewPage(TAKEOFF_PAGE);
            AddNewPage(LANDING_PAGE);
        }
        
        if (_perfPagesEnabled && options.EnableLiveExport)
        {
            // Started before subscribing: the receiver lives as long as the process, so a
            // handler attached ahead of a throwing EnsureStarted would outlive this
            // half-built listener and keep being called on it.
            var receiver = SimExportReceiver.Shared;
            receiver.EnsureStarted();
            receiver.DataReceived += OnLiveExportData;
            _exportReceiver = receiver;
        }
    }

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        switch (_currentPage)
        {
            case TAKEOFF_PAGE:
                if (!Scratchpad.HandleKey(e.Key))
                    HandleTakeoffKey(e.Key);
                break;

            case LANDING_PAGE:
                if (!Scratchpad.HandleKey(e.Key))
                    HandleLandingKey(e.Key);
                break;

            default:
                if (_perfPagesEnabled && e.Key == _perfPageKey)
                { _currentPage = TAKEOFF_PAGE; Compute(); RenderTakeoffPage(); }
                else if (options.A10C.ForwardCduKeysToSim)
                    ForwardCduKeyToSim(e.Key);
                break;
        }
    }

    protected override void InitializeDcsBiosOutputs()
    {
        for (int i = 0; i < 10; i++)
        {
            cduLines[i] = ResolveStr($"CDU_LINE{i}");
        }

        _CONSOLE_BRT = ResolveUInt("INT_CONSOLE_L_BRIGHT");
    }


    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        RegisterTakeoffControls();
        RegisterLandingControls();

        RegisterLight(_CONSOLE_BRT, v =>
            SetCduBacklightBrightnessPercent((int)(v * 100 / _CONSOLE_BRT!.MaxValue)));
        RegisterLight("CDU_BRT", v =>
        {
            var currentBrightness = GetDisplayBrightnessPercent();
            if (v == 0)
                SetCduDisplayBrightnessPercent(Math.Max(0, currentBrightness - BRT_STEP));
            else if (v == 2)
                SetCduDisplayBrightnessPercent(Math.Min(100, currentBrightness + BRT_STEP));
        });

        // --- CDU display lines ---
        bool bottomAligned = options.A10C.DisplayBottomAligned;
        for (int i = 0; i < cduLines.Length; i++)
        {
            int line = bottomAligned ? i + 4 : i;
            RegisterStr(cduLines[i], s => WriteCduLine(line, s));
        }

        if (options.A10C.DisplayCMS)
        {
            int cmsp1Line = bottomAligned ? 0 : 12;
            int cmsp2Line = bottomAligned ? 1 : 13;
            RegisterStr("CMSP1", s => WriteCduLine(cmsp1Line, s));
            RegisterStr("CMSP2", s => WriteCduLine(cmsp2Line, s));
        }
    }

    protected override void RegisterFrontpanelControls()
    {
        RegisterLight(_CONSOLE_BRT, v =>
            FlightDeck.ConsoleBrightness = (byte)(v * 255 / _CONSOLE_BRT!.MaxValue));
        RegisterLight("UFC_INTEN", v =>
        {
            if (v == 0)
                FlightDeck.SegmentBrightnessPercent = Math.Max(0, FlightDeck.SegmentBrightnessPercent - BRT_STEP);
            if (v == 2)
                FlightDeck.SegmentBrightnessPercent = Math.Min(100, FlightDeck.SegmentBrightnessPercent + BRT_STEP);
        });

        RegisterUInt("HDG_DEG", v => FlightDeck.Heading = (int)v);

        RegisterUInt("VVI", v => FlightDeck.VerticalSpeed = ConvertVviToVerticalSpeed((int)v));

        RegisterUInt("ALT_10000FT_CNT", (ctrl, v) =>
        {
            altitudeDigits[2] = ConvertDrumPositionToDigit(v, ctrl.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        RegisterUInt("ALT_1000FT_CNT", (ctrl, v) =>
        {
            altitudeDigits[1] = ConvertDrumPositionToDigit(v, ctrl.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        RegisterUInt("ALT_100FT_CNT", (ctrl, v) =>
        {
            altitudeDigits[0] = ConvertDrumPositionToDigit(v, ctrl.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        RegisterUInt("ALT_100FT", (ctrl, v) =>
        {
            altitudeFine = ConvertPointerPositionToFineAltitude(v, ctrl.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });

        var pressureNames = new[] { "ALT_PRESSURE0", "ALT_PRESSURE1", "ALT_PRESSURE2", "ALT_PRESSURE3" };
        for (int i = 0; i < pressureNames.Length; i++)
        {
            int digitIndex = i;
            RegisterUInt(pressureNames[i], (ctrl, v) =>
            {
                pressureDigits[digitIndex] = ConvertDrumPositionToDigit(v, ctrl.MaxValue);
                FlightDeck.BaroPressure = CombineBaroPressure();
            });
        }

        RegisterUInt("IAS_US_INT", v => FlightDeck.Speed = (int)v);

        RegisterStr("CLOCK_ETC", s =>
        {
            var mode = s.Trim();
            _clockShowsEt = mode.Equals("ET", StringComparison.OrdinalIgnoreCase);
            UpdateClockFields();
        });

        RegisterStr("CLOCK_HH", s =>
        {
            _clockHh = NormalizeTwoDigitClockPart(s);
            UpdateClockFields();
        });

        RegisterStr("CLOCK_MM", s =>
        {
            _clockMm = NormalizeTwoDigitClockPart(s);
            UpdateClockFields();
        });

        RegisterStr("CLOCK_SS", s =>
        {
            _clockSs = NormalizeTwoDigitClockPart(s);
            UpdateClockFields();
        });
    }
    
    private static string ReplaceSpecialChars(string data) =>
        data.Replace("»", "→")
            .Replace("«", "←")
            .Replace("¡", "☐")
            .Replace("®", "Δ")
            .Replace("©", "^")
            .Replace("±", "_")
            .Replace("?", "%")
            .Replace("¶", "⬡");

    
    private void WriteCduLine(int lineIndex, string raw)
    {
        var output = GetCompositor(DEFAULT_PAGE);
        output.Green().Line(lineIndex).WriteLine(ReplaceSpecialChars(raw));

        if (options.A10C.DisplayCMS)
        {
            output.Line(options.A10C.DisplayBottomAligned ? 2 : 11).Amber().WriteLine("------------------------");
        }
    }

    private int CombineBaroPressure()
    {
        // Combine the four digits into inHg format (e.g., 3000 for 30.00 inHg)
        // The FCU expects values >= 2000 for inHg mode (representing 20.00-32.00)
        return pressureDigits[3] * 1000 +
               pressureDigits[2] * 100 +
               pressureDigits[1] * 10 +
               pressureDigits[0];
    }

    private int CombineAltitude()
    {
        // Combine altitude components
        // altitudeDigits[0] contains the visible 100 ft counter digit.
        // altitudeFine contains the pointer-derived 0-99 ft portion inside the current 100 ft band.
        return altitudeDigits[2] * 10000 +
               altitudeDigits[1] * 1000 +
               altitudeDigits[0] * 100 +
               altitudeFine;
    }
    
    private int ConvertDrumPositionToDigit(uint position, int maxValue)
    {
        // A-10C altimeter uses rotating drum displays
        // Each drum rotates through digits 0-9 as the value changes
        // The position value (0-maxValue) represents the angular position of the drum
        // We need to map this to which digit (0-9) is currently visible
        
        // Calculate percentage of full rotation (0.0 to 1.0)
        double rotationPercentage = (double)position / maxValue;
        
        // Each digit occupies 10% of the rotation (0-9 = 10 digits)
        // Use rounding to get the closest digit
        int digit = (int)Math.Round(rotationPercentage * 10) % 10;
        
        return digit;
    }
    
    private int ConvertPointerPositionToFineAltitude(uint position, int maxValue)
    {
        // The 100 ft pointer sweeps through a full 1000 ft each revolution.
        // Use it only for the fine portion inside the current 100 ft drum band.
        double rotationPercentage = (double)position / maxValue;

        int altitudeWithinThousand = (int)Math.Round(rotationPercentage * 1000) % 1000;
        return altitudeWithinThousand % 100;
    }
    
    private int ConvertVviToVerticalSpeed(int rawValue)
    {
        // Variometer linear segments based on DCS LUA definition:
        // Input (ft/min): {-6000, -2000, -1000, 1000, 2000, 6000}
        // Output (gauge): {-1.0, -0.5, -0.29, 0.29, 0.5, 1.0}

        // Convert 0-65535 to -1.0 to 1.0
        double pos = (rawValue / 65535.0) * 2.0 - 1.0;
        double vvi;

        if (pos < -0.5)
            vvi = Interpolate(pos, -1.0, -0.5, -6000, -2000);
        else if (pos < -0.29)
            vvi = Interpolate(pos, -0.5, -0.29, -2000, -1000);
        else if (pos < 0.29)
            vvi = Interpolate(pos, -0.29, 0.29, -1000, 1000);
        else if (pos < 0.5)
            vvi = Interpolate(pos, 0.29, 0.5, 1000, 2000);
        else
            vvi = Interpolate(pos, 0.5, 1.0, 2000, 6000);

        return Math.Clamp((int)vvi, -6000, 6000);

        static double Interpolate(double value, double x1, double x2, double y1, double y2)
        {
            return y1 + (value - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    private void UpdateClockFields()
    {
        var hhmmss = _clockHh + _clockMm + _clockSs;

        // A-10C exposes only the currently selected clock source on CLOCK_HH/MM/SS.
        // Keep UTC updated every time; when ET is selected, also derive elapsed time.
        FlightDeck.ClockUtcTime = hhmmss;

        if (_clockShowsEt)
        {
            FlightDeck.ClockElapsedTime = hhmmss.Substring(2, 4);
        }
        else
        {
            FlightDeck.ClockElapsedTime = string.Empty;
        }
    }

    private static string NormalizeTwoDigitClockPart(string value)
    {
        if (string.IsNullOrWhiteSpace(value)) return "00";

        var trimmed = value.Trim();
        if (trimmed.Length == 1 && char.IsDigit(trimmed[0])) return "0" + trimmed;
        if (trimmed.Length >= 2) return trimmed[..2];
        return "00";
    }

    // Runs on the UDP receiver thread, pre-filling the takeoff page fields the pilot
    // has not typed over. The render timer picks them up on its next tick.
    private void OnLiveExportData(SimExportData data)
    {
        if (!_windManuallySet
            && data.Environment?.WindDirectionDeg.HasValue == true
            && data.Environment?.WindSpeedKts.HasValue == true)
        {
            _wind = $"{data.Environment.WindDirectionDeg}/{(int)data.Environment.WindSpeedKts.Value}";
        }

        if (!_elevManuallySet && data.Position?.AltFt.HasValue == true)
            _elevFt = ((int)Math.Round(data.Position.AltFt.Value)).ToString();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            if (CduDevice != null)
                CduDevice.KeyDown -= HandleKeyDown;

            // The receiver is shared across every listener that wants the feed (other CDUs
            // on the same or a different aircraft may still be reading it), so unsubscribe
            // rather than tearing the socket down.
            if (_exportReceiver != null)
                _exportReceiver.DataReceived -= OnLiveExportData;
            _exportReceiver = null;
        }
        base.Dispose(disposing);
    }
}
