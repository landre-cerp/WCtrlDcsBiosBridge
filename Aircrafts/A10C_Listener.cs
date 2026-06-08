using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;
using WWCduDcsBiosBridge.Frontpanels;

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

    private int speed = 0;
    private int heading = 0;
    private int altitude = 0;
    private int verticalSpeed = 0;
    private int baroPressure = 0;
    private int[] pressureDigits = new int[4];
    private int[] altitudeDigits = new int[3];

    protected override string GetAircraftName() => SupportedAircrafts.A10C_Name;
    protected override string GetFontFile() => "resources/a10c-font-21x31.json";

    public A10C_Listener(
        ICdu? mcdu, 
        UserOptions options,
        FrontpanelHub frontpanelHub) : base(mcdu, SupportedAircrafts.A10C, options, frontpanelHub) {
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

        _HEADING = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("HDG_DEG_MAG");
        _IAS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IAS_US");
        _VS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("VVI");

        _ALT_PRESSURE0 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE0");
        _ALT_PRESSURE1 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE1");
        _ALT_PRESSURE2 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE2");
        _ALT_PRESSURE3 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE3");
    }

    protected override void RegisterLightingControls()
    {
        if (mcdu != null)
        {
            Register(_CONSOLE_BRT, v =>
            {
                mcdu.BacklightBrightnessPercent = (int)(v * 100 / _CONSOLE_BRT!.MaxValue);
                mcdu.RefreshBrightnesses();
            });
            Register(_CDU_BRT, v =>
            {
                if (v == 0)
                    mcdu.DisplayBrightnessPercent = Math.Min(100, mcdu.DisplayBrightnessPercent - BRT_STEP);
                else if (v == 2)
                    mcdu.DisplayBrightnessPercent = Math.Min(100, mcdu.DisplayBrightnessPercent + BRT_STEP);
                mcdu.RefreshBrightnesses();
            });
        }

        if (frontpanelHub.HasFrontpanels)
        {
            Register(_CONSOLE_BRT, v =>
            {
                // Convert to byte range (0-255) directly, not percentage
                var b = (byte)(v * 255 / _CONSOLE_BRT!.MaxValue);
                frontpanelHub.SetBrightness(b, b, b);
            });
        }
    }

    protected override void RegisterMcduControls()
    {
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
        var cap = frontpanelHub.Capabilities;

        if (cap.HasHeadingDisplay)
            Register(_HEADING, v => { heading = (int)v; frontpanelState!.Heading = heading; });

        if (cap.HasVerticalSpeedDisplay)
            Register(_VS, v =>
            {
                verticalSpeed = ConvertVviToVerticalSpeed((int)v);
                frontpanelState!.VerticalSpeed = verticalSpeed;
            });

        if (cap.HasAltitudeDisplay)
        {
            Register(_ALTITUDE_10000ft, v =>
            {
                altitudeDigits[2] = ConvertDrumPositionToDigit(v, _ALTITUDE_10000ft!.MaxValue);
                UpdateAltitude();
                frontpanelState!.Altitude = altitude;
            });
            Register(_ALTITUDE_1000ft, v =>
            {
                altitudeDigits[1] = ConvertDrumPositionToDigit(v, _ALTITUDE_1000ft!.MaxValue);
                UpdateAltitude();
                frontpanelState!.Altitude = altitude;
            });
            Register(_ALTITUDE_100ft, v =>
            {
                altitudeDigits[0] = ConvertDrumPositionToAltitude100ft(v, _ALTITUDE_100ft!.MaxValue);
                UpdateAltitude();
                frontpanelState!.Altitude = altitude;
            });
        }

        if (cap.CanDisplayBarometricPressure)
        {
            var pressureDrums = new[] { _ALT_PRESSURE0, _ALT_PRESSURE1, _ALT_PRESSURE2, _ALT_PRESSURE3 };
            for (int i = 0; i < pressureDrums.Length; i++)
            {
                int digitIndex = i;
                var drum = pressureDrums[i];
                Register(drum, v =>
                {
                    pressureDigits[digitIndex] = ConvertDrumPositionToDigit(v, drum!.MaxValue);
                    if (frontpanelState is FcuEfisState fcuState)
                    {
                        UpdateBaroPressure();
                        fcuState.LeftBaroPressure = baroPressure;
                    }
                });
            }
        }

        if (cap.HasSpeedDisplay)
            RegisterString(_IAS, s =>
            {
                // there's a bug? in DCS-BIOS A-10C module where IAS is 2 knots below the actual value
                var trimmed = s.Trim();
                speed = trimmed == "" ? 0 : int.Parse(trimmed) + 2;
                frontpanelState!.Speed = speed;
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

    // Écrit une ligne CDU (avec substitution des caractères) et redessine le séparateur CMS si activé.
    private void WriteCduLine(int lineIndex, string raw)
    {
        var output = GetCompositor(DEFAULT_PAGE);
        output.Green().Line(lineIndex).WriteLine(ReplaceSpecialChars(raw));

        if (options.DisplayCMS)
        {
            output.Line(options.DisplayBottomAligned ? 2 : 11).Amber().WriteLine("------------------------");
        }
    }

    private void UpdateBaroPressure()
    {
        // Combine the four digits into inHg format (e.g., 3000 for 30.00 inHg)
        // The FCU expects values >= 2000 for inHg mode (representing 20.00-32.00)
        baroPressure = pressureDigits[3] * 1000 + 
                       pressureDigits[2] * 100 + 
                       pressureDigits[1] * 10 + 
                       pressureDigits[0];
    }
    
    private void UpdateAltitude()
    {
        // Combine altitude components
        // altitudeDigits[0] now contains the precise 100s value (0-999)
        // altitudeDigits[1] and [2] contain single digits (0-9)
        altitude = altitudeDigits[2] * 10000 + 
                   altitudeDigits[1] * 1000 + 
                   altitudeDigits[0];
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
    
    private int ConvertDrumPositionToAltitude100ft(uint position, int maxValue)
    {
        // The 100ft drum provides continuous position data
        // We can use the fractional position to get 20ft precision
        // Position 0-maxValue maps to 0-1000 feet (full rotation shows 0,1,2...9 then back to 0)
        double rotationPercentage = (double)position / maxValue;
        
        // Convert to altitude: 0.0 = 0ft, 1.0 = 1000ft
        // But we only want the hundreds portion (0-900)
        int altitude100 = (int)(rotationPercentage * 1000) % 1000;
        
        return altitude100;
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
}
