using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using System.Globalization;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class F16C_Nav_Page
{
    private DCSBIOSOutput? _EHSI_COURSE;
    private DCSBIOSOutput? _EHSI_RANGE;
    private DCSBIOSOutput? _EHSI_MODE_LEFT;
    private DCSBIOSOutput? _EHSI_MODE_RIGHT;

    private DCSBIOSOutput? _EHSI_RANGE_INVALID;
    private DCSBIOSOutput? _EHSI_CRS_SET_KNB;
    private DCSBIOSOutput? _EHSI_HDG_SET_KNB;
    private DCSBIOSOutput? _STANDBY_COMPASS_HEADING;

    private DCSBIOSOutput? _ALT_10000;
    private DCSBIOSOutput? _ALT_1000;
    private DCSBIOSOutput? _ALT_100;
    private DCSBIOSOutput? _ALT_PNEU;
    private DCSBIOSOutput? _QNH_D0;
    private DCSBIOSOutput? _QNH_D1;
    private DCSBIOSOutput? _QNH_D2;
    private DCSBIOSOutput? _QNH_D3;

    private DCSBIOSOutput? _AIRSPEED;
    private DCSBIOSOutput? _MACH;
    private DCSBIOSOutput? _VVI;

    private DCSBIOSOutput? _FUEL_TOT_10K;
    private DCSBIOSOutput? _FUEL_TOT_1K;
    private DCSBIOSOutput? _FUEL_TOT_100;

    private DCSBIOSOutput? _FUEL_FF_10K;
    private DCSBIOSOutput? _FUEL_FF_1K;
    private DCSBIOSOutput? _FUEL_FF_100;

    private DCSBIOSOutput? _FUEL_AL;
    private DCSBIOSOutput? _FUEL_FR;
    private DCSBIOSOutput? _LIGHT_FUEL_LOW;

    private DCSBIOSOutput? _CLOCK_H;
    private DCSBIOSOutput? _CLOCK_MS;
    private DCSBIOSOutput? _CLOCK_ELAPSED_M;
    private DCSBIOSOutput? _CLOCK_ELAPSED_S;

    private string _ehsiCourse = "";
    private string _ehsiRange = "";
    private string _ehsiModeLeft = "";
    private string _ehsiModeRight = "";
    private bool _ehsiRangeInvalid;

    private int _currentHeadingDeg;
    private int _selectedCourseDeg;
    private int _hdgBugDeg;
    private uint _lastHdgKnobValue;
    private bool _rangeInvalid;

    private int _altFt;
    private int _altD10k;
    private int _altD1k;
    private int _altLowFt;
    private int _iasKts;
    private double _mach;
    private int _vviFpm;
    private int _fuelTotalLb;
    private int _fuelFlowPph;
    private int _fuelTot10kDigit;
    private int _fuelTot1kDigit;
    private int _fuelTot100Digit;
    private int _fuelFf10kDigit;
    private int _fuelFf1kDigit;
    private int _fuelFf100Digit;
    private int _fuelAlLb;
    private int _fuelFrLb;
    private bool _fuelLow;
    private bool _pneuFail;
    private int _qnhD0;
    private int _qnhD1;
    private int _qnhD2;
    private int _qnhD3;
    private int _clockH;
    private int _clockMin;
    private int _elapsedMin;
    private int _elapsedSec;

    public void InitializeControls()
    {
        _EHSI_COURSE = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_COURSE");
        _EHSI_RANGE = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_RANGE");
        _EHSI_MODE_LEFT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_MODE_LEFT");
        _EHSI_MODE_RIGHT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_MODE_RIGHT");

        _EHSI_RANGE_INVALID = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_RANGE_INVALID");
        _EHSI_CRS_SET_KNB = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_CRS_SET_KNB");
        _EHSI_HDG_SET_KNB = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_HDG_SET_KNB");
        _STANDBY_COMPASS_HEADING = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("STANDBY_COMPASS_HEADING");

        _ALT_10000 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_10000_FT_CNT");
        _ALT_1000 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_1000_FT_CNT");
        _ALT_100 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_100_FT_CNT");
        _ALT_PNEU = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PNEU_FLAG");
        _QNH_D0 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_0_CNT");
        _QNH_D1 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_1_CNT");
        _QNH_D2 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_2_CNT");
        _QNH_D3 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_3_CNT");

        _AIRSPEED = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("AIRSPEED");
        _MACH = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MACH_INDICATOR");
        _VVI = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("VVI");

        _FUEL_TOT_10K = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_10K");
        _FUEL_TOT_1K = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_1K");
        _FUEL_TOT_100 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_100");

        _FUEL_FF_10K = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_10K");
        _FUEL_FF_1K = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_1K");
        _FUEL_FF_100 = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_100");

        _FUEL_AL = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUEL_AL");
        _FUEL_FR = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUEL_FR");
        _LIGHT_FUEL_LOW = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_AFT_FUEL_LOW");

        _CLOCK_H = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_CURRTIME_H");
        _CLOCK_MS = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_CURRTIME_MS");
        _CLOCK_ELAPSED_M = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_ELAPSED_TIME_M");
        _CLOCK_ELAPSED_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_ELAPSED_TIME_SEC");
    }

    public bool ProcessData(DCSBIOSDataEventArgs e)
    {
        bool refreshDisplay = false;

        if (_STANDBY_COMPASS_HEADING != null && e.Address == _STANDBY_COMPASS_HEADING.Address)
        {
            uint raw = _STANDBY_COMPASS_HEADING.GetUIntValue(e.Data);
            _currentHeadingDeg = (int)Math.Round(raw * 360.0 / 65536.0, MidpointRounding.AwayFromZero) % 360;
            refreshDisplay = true;
        }

        if (_EHSI_RANGE_INVALID != null && e.Address == _EHSI_RANGE_INVALID.Address)
        {
            _rangeInvalid = _EHSI_RANGE_INVALID.GetUIntValue(e.Data) == 1;
        }

        if (_EHSI_HDG_SET_KNB != null && e.Address == _EHSI_HDG_SET_KNB.Address)
        {
            uint newVal = _EHSI_HDG_SET_KNB.GetUIntValue(e.Data);
            int delta = (int)newVal - (int)_lastHdgKnobValue;
            if (Math.Abs(delta) < 32768)
            {
                _hdgBugDeg = (_hdgBugDeg + Math.Sign(delta) + 360) % 360;
            }

            _lastHdgKnobValue = newVal;
        }

        if (_EHSI_RANGE_INVALID != null && e.Address == _EHSI_RANGE_INVALID.Address)
        {
            _ehsiRangeInvalid = _EHSI_RANGE_INVALID.GetUIntValue(e.Data) == 1;
            refreshDisplay = true;
        }

        if (_EHSI_CRS_SET_KNB != null && e.Address == _EHSI_CRS_SET_KNB.Address)
        {
            _selectedCourseDeg = KnobToDegrees(_EHSI_CRS_SET_KNB.GetUIntValue(e.Data));
            refreshDisplay = true;
        }

        if (_ALT_10000 != null && e.Address == _ALT_10000.Address)
        {
            _altD10k = DecodeDrumDigit(_ALT_10000, e.Data);
            _altFt = _altD10k * 10000 + _altD1k * 1000 + _altLowFt;
            refreshDisplay = true;
        }

        if (_ALT_1000 != null && e.Address == _ALT_1000.Address)
        {
            _altD1k = DecodeDrumDigit(_ALT_1000, e.Data);
            _altFt = _altD10k * 10000 + _altD1k * 1000 + _altLowFt;
            refreshDisplay = true;
        }

        if (_ALT_100 != null && e.Address == _ALT_100.Address)
        {
            _altLowFt = DecodeSubThousandValue(_ALT_100, e.Data, 1);
            _altFt = _altD10k * 10000 + _altD1k * 1000 + _altLowFt;
            refreshDisplay = true;
        }

        if (_ALT_PNEU != null && e.Address == _ALT_PNEU.Address)
        {
            _pneuFail = _ALT_PNEU.GetUIntValue(e.Data) > 32767;
            refreshDisplay = true;
        }

        if (_QNH_D0 != null && e.Address == _QNH_D0.Address) { _qnhD0 = DecodeDrumDigit(_QNH_D0, e.Data); refreshDisplay = true; }
        if (_QNH_D1 != null && e.Address == _QNH_D1.Address) { _qnhD1 = DecodeDrumDigit(_QNH_D1, e.Data); refreshDisplay = true; }
        if (_QNH_D2 != null && e.Address == _QNH_D2.Address) { _qnhD2 = DecodeDrumDigit(_QNH_D2, e.Data); refreshDisplay = true; }
        if (_QNH_D3 != null && e.Address == _QNH_D3.Address) { _qnhD3 = DecodeDrumDigit(_QNH_D3, e.Data); refreshDisplay = true; }

        if (_AIRSPEED != null && e.Address == _AIRSPEED.Address)
        {
            _iasKts = (int)Math.Round(_AIRSPEED.GetUIntValue(e.Data) * 1000.0 / 65535.0);
            refreshDisplay = true;
        }

        if (_MACH != null && e.Address == _MACH.Address)
        {
            double rawMachNeedle = _MACH.GetUIntValue(e.Data) / (double)_MACH.MaxValue;
            _mach = DecodeMach(rawMachNeedle);
            refreshDisplay = true;
        }

        if (_VVI != null && e.Address == _VVI.Address)
        {
            _vviFpm = (int)Math.Round((_VVI.GetUIntValue(e.Data) - 32768.0) * 6000.0 / 32767.0);
            refreshDisplay = true;
        }

        if (_FUEL_TOT_10K != null && e.Address == _FUEL_TOT_10K.Address)
        {
            _fuelTot10kDigit = DecodeDrumDigit(_FUEL_TOT_10K, e.Data);
            _fuelTotalLb = _fuelTot10kDigit * 10000 + _fuelTot1kDigit * 1000 + _fuelTot100Digit * 100;
            refreshDisplay = true;
        }

        if (_FUEL_TOT_1K != null && e.Address == _FUEL_TOT_1K.Address)
        {
            _fuelTot1kDigit = DecodeDrumDigit(_FUEL_TOT_1K, e.Data);
            _fuelTotalLb = _fuelTot10kDigit * 10000 + _fuelTot1kDigit * 1000 + _fuelTot100Digit * 100;
            refreshDisplay = true;
        }

        if (_FUEL_TOT_100 != null && e.Address == _FUEL_TOT_100.Address)
        {
            _fuelTot100Digit = DecodeDrumDigit(_FUEL_TOT_100, e.Data);
            _fuelTotalLb = _fuelTot10kDigit * 10000 + _fuelTot1kDigit * 1000 + _fuelTot100Digit * 100;
            refreshDisplay = true;
        }

        if (_FUEL_FF_10K != null && e.Address == _FUEL_FF_10K.Address)
        {
            _fuelFf10kDigit = DecodeDrumDigit(_FUEL_FF_10K, e.Data);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            refreshDisplay = true;
        }

        if (_FUEL_FF_1K != null && e.Address == _FUEL_FF_1K.Address)
        {
            _fuelFf1kDigit = DecodeDrumDigit(_FUEL_FF_1K, e.Data);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            refreshDisplay = true;
        }

        if (_FUEL_FF_100 != null && e.Address == _FUEL_FF_100.Address)
        {
            _fuelFf100Digit = DecodeSubThousandValue(_FUEL_FF_100, e.Data, 50);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            refreshDisplay = true;
        }

        if (_FUEL_AL != null && e.Address == _FUEL_AL.Address)
        {
            _fuelAlLb = (int)Math.Round(_FUEL_AL.GetUIntValue(e.Data) * 4000.0 / 65535.0);
            refreshDisplay = true;
        }

        if (_FUEL_FR != null && e.Address == _FUEL_FR.Address)
        {
            _fuelFrLb = (int)Math.Round(_FUEL_FR.GetUIntValue(e.Data) * 4000.0 / 65535.0);
            refreshDisplay = true;
        }

        if (_LIGHT_FUEL_LOW != null && e.Address == _LIGHT_FUEL_LOW.Address)
        {
            _fuelLow = _LIGHT_FUEL_LOW.GetUIntValue(e.Data) == 1;
            refreshDisplay = true;
        }

        if (_CLOCK_H != null && e.Address == _CLOCK_H.Address)
        {
            int rawHour = (int)Math.Floor(_CLOCK_H.GetUIntValue(e.Data) * 24.0 / (_CLOCK_H.MaxValue + 1.0));
            _clockH = (rawHour + 23) % 24;
            refreshDisplay = true;
        }

        if (_CLOCK_MS != null && e.Address == _CLOCK_MS.Address)
        {
            _clockMin = (int)Math.Floor(_CLOCK_MS.GetUIntValue(e.Data) * 60.0 / 65536.0);
            refreshDisplay = true;
        }

        if (_CLOCK_ELAPSED_M != null && e.Address == _CLOCK_ELAPSED_M.Address)
        {
            _elapsedMin = (int)Math.Floor(_CLOCK_ELAPSED_M.GetUIntValue(e.Data) * 60.0 / 65536.0);
            refreshDisplay = true;
        }

        if (_CLOCK_ELAPSED_S != null && e.Address == _CLOCK_ELAPSED_S.Address)
        {
            _elapsedSec = (int)Math.Floor(_CLOCK_ELAPSED_S.GetUIntValue(e.Data) * 60.0 / 65536.0);
            refreshDisplay = true;
        }

        return refreshDisplay;
    }

    public bool ProcessData(DCSBIOSStringDataEventArgs e)
    {
        bool hsiChanged = false;

        if (_EHSI_COURSE != null && e.Address == _EHSI_COURSE.Address)
        {
            _ehsiCourse = e.StringData.Trim();
            if (int.TryParse(e.StringData.Trim(), out int crs))
            {
                _selectedCourseDeg = crs;
            }

            hsiChanged = true;
        }

        if (_EHSI_RANGE != null && e.Address == _EHSI_RANGE.Address)
        {
            string raw = e.StringData.Replace("\0", "").Trim();
            string normalized = NormalizeRangeText(raw);
            if (int.TryParse(normalized, out int rangeRaw))
            {
                _ehsiRange = (rangeRaw / 10.0).ToString("F1", CultureInfo.InvariantCulture);
            }
            else if (double.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out double rangeNm))
            {
                _ehsiRange = rangeNm.ToString("F1", CultureInfo.InvariantCulture);
            }
            else
            {
                _ehsiRange = normalized;
            }

            hsiChanged = true;
        }

        if (_EHSI_MODE_LEFT != null && e.Address == _EHSI_MODE_LEFT.Address)
        {
            _ehsiModeLeft = e.StringData.Trim();
            hsiChanged = true;
        }

        if (_EHSI_MODE_RIGHT != null && e.Address == _EHSI_MODE_RIGHT.Address)
        {
            _ehsiModeRight = e.StringData.Trim();
            hsiChanged = true;
        }

        return hsiChanged;
    }

    public void Render(Compositor o)
    {
        o.Line(0).Amber().WriteLine($"F-16C NAV  STDBY HDG:{_currentHeadingDeg:D3}");

        string rng = _ehsiRangeInvalid ? "---" : (_ehsiRange.Length > 0 ? _ehsiRange : "---");
        string crs = _ehsiCourse.Length > 0 ? _ehsiCourse : "---";
        o.Line(1).Green().WriteLine($"RNG:{rng,6}nm    CRS:{crs,3}");

        string modeL = _ehsiModeLeft.Length > 0 ? _ehsiModeLeft : "   ";
        string modeR = _ehsiModeRight.Length > 0 ? _ehsiModeRight : "NAV";
        o.Line(2).Green().WriteLine($"MODE:{modeL} {modeR,-18}");

        o.Line(3).Green().WriteLine(new string('-', 24));

        string vs = _vviFpm >= 0 ? $"+{_vviFpm,5}" : $"{_vviFpm,6}";
        o.Line(4).Green().WriteLine($"ALT:{_altFt,6}ft VS:{vs}");

        o.Line(5).Green().WriteLine($"IAS:{_iasKts,6}kt MCH:{_mach:F2}");

        string qnh = $"{_qnhD3}{_qnhD2}.{_qnhD1}{_qnhD0}";
        string pneu = _pneuFail ? "FAIL" : " OK ";
        o.Line(6).Green().WriteLine($"QNH:{qnh,6}in PNU:{pneu}");

        o.Line(7).Green().WriteLine(new string('-', 24));

        o.Line(8).Green().WriteLine($"TOT:{_fuelTotalLb,5}lb FF:{_fuelFlowPph,5}");
        o.Line(9).Green().WriteLine($"AFT:{_fuelAlLb,5}lb FWD:{_fuelFrLb,4}");

        double endurH = _fuelFlowPph > 0 ? (double)_fuelTotalLb / _fuelFlowPph : 0;
        int endurMin = (int)(endurH * 60);
        int rangeNm = (int)(endurH * 300);
        o.Line(10).Green().WriteLine($"END:{endurMin / 60,2}h{endurMin % 60,2}   RNG~{rangeNm,4}nm");

        o.Line(11).Green().WriteLine(new string('-', 24));
        o.Line(12).Green().WriteLine($"TIME:{_clockH:D2}:{_clockMin:D2}Z  HACK:{_elapsedMin:D2}:{_elapsedSec:D2}");

        if (_fuelLow)
        {
            o.Line(13).Amber().WriteLine("** FUEL LOW **      (NAV)");
        }
        else
        {
            o.Line(13).Green().WriteLine($"{"(NAV)",24}");
        }
    }

    private static int KnobToDegrees(uint raw) =>
        (int)Math.Round(raw / 65535.0 * 359.0) % 360;

    private static int DecodeDrumDigit(DCSBIOSOutput output, uint data)
    {
        if (output.MaxValue == 0)
        {
            return 0;
        }

        double normalized = output.GetUIntValue(data) / (double)output.MaxValue;
        int digit = (int)Math.Floor(normalized * 10.0 + 0.05);
        return digit == 10 ? 0 : Math.Clamp(digit, 0, 9);
    }

    private static double DecodeMach(double rawNeedle)
    {
        const double rawAtMachOne = 0.92;
        double clamped = Math.Max(0.0, rawNeedle);
        return clamped / rawAtMachOne;
    }

    private static int ComposeFuelFlowPph(int tenThousandsDigit, int thousandsDigit, int hundredsDigit)
    {
        return tenThousandsDigit * 10000 + thousandsDigit * 1000 + hundredsDigit;
    }

    private static int DecodeSubThousandValue(DCSBIOSOutput output, uint data, int step)
    {
        if (output.MaxValue == 0 || step <= 0)
        {
            return 0;
        }

        double scaled = output.GetUIntValue(data) * 1000.0 / output.MaxValue;
        int quantized = (int)Math.Round(scaled / step, MidpointRounding.AwayFromZero) * step;
        return Math.Clamp(quantized, 0, 1000 - step);
    }

    private static string NormalizeRangeText(string raw)
    {
        if (string.IsNullOrWhiteSpace(raw))
        {
            return string.Empty;
        }

        string cleaned = raw.Replace(" ", "").Trim();
        if (cleaned.EndsWith("NM", StringComparison.OrdinalIgnoreCase))
        {
            cleaned = cleaned[..^2];
        }

        return cleaned.Trim();
    }
}
