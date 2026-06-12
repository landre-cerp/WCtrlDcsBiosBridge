using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class A10C_Listener : AircraftListener
{
    private const int BRT_STEP = 5;
    private readonly DCSBIOSOutput?[] cduLines = new DCSBIOSOutput?[10];

    private DCSBIOSOutput? _CDU_BRT; 
    private DCSBIOSOutput? _MASTER_CAUTION; 

    private DCSBIOSOutput? _CONSOLE_BRT; 
    private DCSBIOSOutput? _NOSE_SW_GREENLIGHT;
    private DCSBIOSOutput? _CANOPY_LED; 
    private DCSBIOSOutput? _GUN_READY;

    private DCSBIOSOutput? _CMSP1;
    private DCSBIOSOutput? _CMSP2;

    private DCSBIOSOutput? _HEADING;
    private DCSBIOSOutput? _IAS;
    private DCSBIOSOutput? _VS;
    private DCSBIOSOutput? _ALT_PRESSURE0;
    private DCSBIOSOutput? _ALT_PRESSURE1;
    private DCSBIOSOutput? _ALT_PRESSURE2;
    private DCSBIOSOutput? _ALT_PRESSURE3;

    private DCSBIOSOutput? _ALTITUDE_10000ft;
    private DCSBIOSOutput? _ALTITUDE_1000ft;
    private DCSBIOSOutput? _ALTITUDE_100ft;
    private DCSBIOSOutput? _ALTITUDE_100ftPointer;

    private DCSBIOSOutput? _GEAR_L_SAFE;
    private DCSBIOSOutput? _GEAR_R_SAFE;
    private DCSBIOSOutput? _GEAR_N_SAFE;

    private DCSBIOSOutput? _HANDLE_GEAR_WARNING;
    private DCSBIOSOutput? _UFC_INTEN;

    private DCSBIOSOutput? _CLOCK_ETC;
    private DCSBIOSOutput? _CLOCK_HH;
    private DCSBIOSOutput? _CLOCK_MM;
    private DCSBIOSOutput? _CLOCK_SS;

    private int[] pressureDigits = new int[4];
    private int[] altitudeDigits = new int[3];
    private int altitudeFine;

    private bool _clockShowsEt;
    private string _clockHh = "00";
    private string _clockMm = "00";
    private string _clockSs = "00";

    public A10C_Listener(
        ICdu? mcdu,
        UserOptions options) : base(mcdu, AircraftRegistry.A10C, options) {
    }

    ~A10C_Listener()
    {
        Dispose(false);
    }

    protected override void InitializeDcsBiosOutputs()
    {
        for (int i = 0; i < 10; i++)
        {
            cduLines[i] = DCSBIOSControlLocator.GetStringDCSBIOSOutput($"CDU_LINE{i}");
        }

        _CDU_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CDU_BRT");
        _MASTER_CAUTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MASTER_CAUTION");

        _CONSOLE_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("INT_CONSOLE_L_BRIGHT");
        _NOSE_SW_GREENLIGHT= DCSBIOSControlLocator.GetUIntDCSBIOSOutput("NOSEWHEEL_STEERING");
        _CANOPY_LED = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CANOPY_UNLOCKED");
        _GUN_READY = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("GUN_READY");

        _CMSP1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMSP1");
        _CMSP2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMSP2");

        _ALTITUDE_10000ft = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_10000FT_CNT");
        _ALTITUDE_1000ft = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_1000FT_CNT");
        _ALTITUDE_100ft = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_100FT_CNT");
        _ALTITUDE_100ftPointer = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_100FT");

        _HEADING = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("HDG_DEG");
        _IAS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("IAS_US_INT");
        _VS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("VVI");

        _ALT_PRESSURE0 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE0");
        _ALT_PRESSURE1 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE1");
        _ALT_PRESSURE2 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE2");
        _ALT_PRESSURE3 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE3");

        _GEAR_L_SAFE = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("GEAR_L_SAFE");
        _GEAR_R_SAFE = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("GEAR_R_SAFE");
        _GEAR_N_SAFE = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("GEAR_N_SAFE");

        _HANDLE_GEAR_WARNING = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("HANDLE_GEAR_WARNING");

        _UFC_INTEN = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("UFC_INTEN");

        _CLOCK_ETC = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_ETC");
        _CLOCK_HH = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_HH");
        _CLOCK_MM = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_MM");
        _CLOCK_SS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CLOCK_SS");
    }


    protected override void RegisterCduControls()
    {
        if (!options.DisableLightingManagement && mcdu != null) { 
            Register(_CONSOLE_BRT, v =>
            {
                mcdu!.BacklightBrightnessPercent = (int)(v * 100 / _CONSOLE_BRT!.MaxValue);
                mcdu!.RefreshBrightnesses();
            });
            Register(_CDU_BRT, v =>
            {
                if (v == 0)
                    mcdu!.DisplayBrightnessPercent = Math.Min(100, mcdu!.DisplayBrightnessPercent - BRT_STEP);
                else if (v == 2)
                    mcdu!.DisplayBrightnessPercent = Math.Min(100, mcdu!.DisplayBrightnessPercent + BRT_STEP);
                mcdu!.RefreshBrightnesses();
            }); 
        }

        // --- LEDs ---
        Register(_CANOPY_LED,         v => { mcdu!.Leds.Fm2  = v == 1; mcdu!.RefreshLeds(); });
        Register(_NOSE_SW_GREENLIGHT, v => { mcdu!.Leds.Ind  = v == 1; mcdu!.RefreshLeds(); });
        Register(_GUN_READY,          v => { mcdu!.Leds.Fm1  = v == 1; mcdu!.RefreshLeds(); });
        Register(_MASTER_CAUTION,     v => { mcdu!.Leds.Fail = v == 1; mcdu!.RefreshLeds(); });

        // --- CDU display lines ---
        bool bottomAligned = options.DisplayBottomAligned;
        for (int i = 0; i < cduLines.Length; i++)
        {
            int line = bottomAligned ? i + 4 : i;
            RegisterString(cduLines[i], s => WriteCduLine(line, s));
        }

        if (options.DisplayCMS)
        {
            int cmsp1Line = bottomAligned ? 0 : 12;
            int cmsp2Line = bottomAligned ? 1 : 13;
            RegisterString(_CMSP1, s => WriteCduLine(cmsp1Line, s));
            RegisterString(_CMSP2, s => WriteCduLine(cmsp2Line, s));
        }
    }

    protected override void RegisterFrontpanelControls()
    {
        if (!options.DisableLightingManagement)
        {
            Register(_CONSOLE_BRT, v =>
            {
                // Convert to byte range (0-255) directly, not percentage
                FlightDeck.ConsoleBrightness = (byte)(v * 255 / _CONSOLE_BRT!.MaxValue);
            });

            Register(_UFC_INTEN, v =>
            {
                if (v == 0)
                {
                    FlightDeck.SegmentBrightnessPercent = Math.Min(100, FlightDeck.SegmentBrightnessPercent - BRT_STEP);
                }
                if (v==2)
                {
                    FlightDeck.SegmentBrightnessPercent = Math.Min(100, FlightDeck.SegmentBrightnessPercent + BRT_STEP);
                }

            });
        }

        Register(_HEADING, v => FlightDeck.Heading = (int)v);

        Register(_VS, v => FlightDeck.VerticalSpeed = ConvertVviToVerticalSpeed((int)v));

        Register(_ALTITUDE_10000ft, v =>
        {
            altitudeDigits[2] = ConvertDrumPositionToDigit(v, _ALTITUDE_10000ft!.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        Register(_ALTITUDE_1000ft, v =>
        {
            altitudeDigits[1] = ConvertDrumPositionToDigit(v, _ALTITUDE_1000ft!.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        Register(_ALTITUDE_100ft, v =>
        {
            altitudeDigits[0] = ConvertDrumPositionToDigit(v, _ALTITUDE_100ft!.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });
        Register(_ALTITUDE_100ftPointer, v =>
        {
            altitudeFine = ConvertPointerPositionToFineAltitude(v, _ALTITUDE_100ftPointer!.MaxValue);
            FlightDeck.Altitude = CombineAltitude();
        });

        var pressureDrums = new[] { _ALT_PRESSURE0, _ALT_PRESSURE1, _ALT_PRESSURE2, _ALT_PRESSURE3 };
        for (int i = 0; i < pressureDrums.Length; i++)
        {
            int digitIndex = i;
            var drum = pressureDrums[i];
            Register(drum, v =>
            {
                pressureDigits[digitIndex] = ConvertDrumPositionToDigit(v, drum!.MaxValue);
                FlightDeck.BaroPressure = CombineBaroPressure();
            });
        }

        Register(_IAS, s =>
        {
            FlightDeck.Speed = (int)s;
        });

        Register(_GEAR_L_SAFE, v => FlightDeck.GearLeftDown = v == 1);
        Register(_GEAR_N_SAFE, v => FlightDeck.GearNoseDown = v == 1);
        Register(_GEAR_R_SAFE, v => FlightDeck.GearRightDown = v == 1);
        Register(_HANDLE_GEAR_WARNING, v => FlightDeck.GearWarning = v == 1);

        RegisterString(_CLOCK_ETC, s =>
        {
            var mode = s.Trim();
            _clockShowsEt = mode.Equals("ET", StringComparison.OrdinalIgnoreCase);
            UpdateAgp32ClockFields();
        });

        RegisterString(_CLOCK_HH, s =>
        {
            _clockHh = NormalizeTwoDigitClockPart(s);
            UpdateAgp32ClockFields();
        });

        RegisterString(_CLOCK_MM, s =>
        {
            _clockMm = NormalizeTwoDigitClockPart(s);
            UpdateAgp32ClockFields();
        });

        RegisterString(_CLOCK_SS, s =>
        {
            _clockSs = NormalizeTwoDigitClockPart(s);
            UpdateAgp32ClockFields();
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

        if (options.DisplayCMS)
        {
            output.Line(options.DisplayBottomAligned ? 2 : 11).Amber().WriteLine("------------------------");
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

    private void UpdateAgp32ClockFields()
    {
        var hhmmss = _clockHh + _clockMm + _clockSs;

        // A-10C exposes only the currently selected clock source on CLOCK_HH/MM/SS.
        // Keep UTC updated every time; when ET is selected, also derive AGP32 ET.
        FlightDeck.Agp32UtcTime = hhmmss;

        if (_clockShowsEt)
        {
            // AGP32 ET has 4 digits; keep minutes+seconds from HHMMSS.
            FlightDeck.Agp32Et = hhmmss.Substring(2, 4);
        }
        else
        {
            FlightDeck.Agp32Et = string.Empty;
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
}
