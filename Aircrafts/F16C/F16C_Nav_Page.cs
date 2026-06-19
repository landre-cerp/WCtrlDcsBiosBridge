using System.Globalization;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class F16C_Listener
{
    private string _ehsiCourse    = "";
    private string _ehsiRange     = "";
    private string _ehsiModeLeft  = "";
    private string _ehsiModeRight = "";
    private bool   _ehsiRangeInvalid;
    private bool   _rangeInvalid;

    private int    _currentHeadingDeg;
    private int    _selectedCourseDeg;

    private int    _altFt;
    private int    _altD10k;
    private int    _altD1k;
    private int    _altLowFt;
    private int    _iasKts;
    private double _mach;
    private int    _vviFpm;
    private int    _fuelTotalLb;
    private int    _fuelFlowPph;
    private int    _fuelTot10kDigit;
    private int    _fuelTot1kDigit;
    private int    _fuelTot100Digit;
    private int    _fuelFf10kDigit;
    private int    _fuelFf1kDigit;
    private int    _fuelFf100Digit;
    private int    _fuelAlLb;
    private int    _fuelFrLb;
    private bool   _fuelLow;
    private bool   _pneuFail;
    private int    _qnhD0;
    private int    _qnhD1;
    private int    _qnhD2;
    private int    _qnhD3;
    private int    _clockH;
    private int    _clockMin;
    private int    _elapsedMin;
    private int    _elapsedSec;

    private void RegisterNavControls()
    {
        RegisterUInt("STANDBY_COMPASS_HEADING", v =>
        {
            _currentHeadingDeg = (int)Math.Round(v * 360.0 / 65536.0, MidpointRounding.AwayFromZero) % 360;
            RenderNav();
        });

        RegisterUInt("EHSI_RANGE_INVALID", v =>
        {
            _ehsiRangeInvalid = v == 1;
            _rangeInvalid = v == 1;
            RenderNav();
        });

        RegisterUInt("EHSI_CRS_SET_KNB", v =>
        {
            _selectedCourseDeg = KnobToDegrees(v);
            RenderNav();
        });

        RegisterUInt("ALT_10000_FT_CNT", (ctrl, v) =>
        {
            _altD10k = DecodeDrumDigit(v, ctrl.MaxValue);
            RecomputeAltitude();
            RenderNav();
        });

        RegisterUInt("ALT_1000_FT_CNT", (ctrl, v) =>
        {
            _altD1k = DecodeDrumDigit(v, ctrl.MaxValue);
            RecomputeAltitude();
            RenderNav();
        });

        RegisterUInt("ALT_100_FT_CNT", (ctrl, v) =>
        {
            _altLowFt = DecodeSubThousandValue(v, ctrl.MaxValue, 1);
            RecomputeAltitude();
            RenderNav();
        });

        RegisterUInt("ALT_PNEU_FLAG", v =>
        {
            _pneuFail = v > 32767;
            RenderNav();
        });

        RegisterUInt("ALT_PRESSURE_DRUM_0_CNT", (ctrl, v) => { _qnhD0 = DecodeDrumDigit(v, ctrl.MaxValue); RenderNav(); });
        RegisterUInt("ALT_PRESSURE_DRUM_1_CNT", (ctrl, v) => { _qnhD1 = DecodeDrumDigit(v, ctrl.MaxValue); RenderNav(); });
        RegisterUInt("ALT_PRESSURE_DRUM_2_CNT", (ctrl, v) => { _qnhD2 = DecodeDrumDigit(v, ctrl.MaxValue); RenderNav(); });
        RegisterUInt("ALT_PRESSURE_DRUM_3_CNT", (ctrl, v) => { _qnhD3 = DecodeDrumDigit(v, ctrl.MaxValue); RenderNav(); });

        RegisterUInt("AIRSPEED", (ctrl, v) =>
        {
            _iasKts = (int)Math.Round(DecodeAirspeed(v / (double)ctrl.MaxValue));
            RenderNav();
        });

        RegisterUInt("MACH_INDICATOR", (ctrl, v) =>
        {
            _mach = DecodeMach(v / (double)ctrl.MaxValue);
            RenderNav();
        });

        RegisterUInt("VVI", v =>
        {
            _vviFpm = DecodeVviFpm(v);
            RenderNav();
        });

        RegisterUInt("FUELTOTALIZER_10K", (ctrl, v) =>
        {
            _fuelTot10kDigit = DecodeDrumDigit(v, ctrl.MaxValue);
            RecomputeFuelTotal();
            RenderNav();
        });

        RegisterUInt("FUELTOTALIZER_1K", (ctrl, v) =>
        {
            _fuelTot1kDigit = DecodeDrumDigit(v, ctrl.MaxValue);
            RecomputeFuelTotal();
            RenderNav();
        });

        RegisterUInt("FUELTOTALIZER_100", (ctrl, v) =>
        {
            _fuelTot100Digit = DecodeDrumDigit(v, ctrl.MaxValue);
            RecomputeFuelTotal();
            RenderNav();
        });

        RegisterUInt("FUELFLOWCOUNTER_10K", (ctrl, v) =>
        {
            _fuelFf10kDigit = DecodeDrumDigit(v, ctrl.MaxValue);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            RenderNav();
        });

        RegisterUInt("FUELFLOWCOUNTER_1K", (ctrl, v) =>
        {
            _fuelFf1kDigit = DecodeDrumDigit(v, ctrl.MaxValue);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            RenderNav();
        });

        RegisterUInt("FUELFLOWCOUNTER_100", (ctrl, v) =>
        {
            _fuelFf100Digit = DecodeSubThousandValue(v, ctrl.MaxValue, 50);
            _fuelFlowPph = ComposeFuelFlowPph(_fuelFf10kDigit, _fuelFf1kDigit, _fuelFf100Digit);
            RenderNav();
        });

        RegisterUInt("FUEL_AL", v =>
        {
            _fuelAlLb = (int)Math.Round(v * 4000.0 / 65535.0);
            RenderNav();
        });

        RegisterUInt("FUEL_FR", v =>
        {
            _fuelFrLb = (int)Math.Round(v * 4000.0 / 65535.0);
            RenderNav();
        });

        RegisterUInt("LIGHT_AFT_FUEL_LOW", v =>
        {
            _fuelLow = v == 1;
            RenderNav();
        });

        RegisterUInt("CLOCK_CURRTIME_H", v =>
        {
            int hour12 = (int)Math.Floor(v * 12.0 / 65536.0) % 12;
            _clockH = hour12 == 0 ? 12 : hour12;
            RenderNav();
        });

        RegisterUInt("CLOCK_CURRTIME_MS", v =>
        {
            _clockMin = (int)Math.Floor(v * 60.0 / 65536.0);
            RenderNav();
        });

        RegisterUInt("CLOCK_ELAPSED_TIME_M", v =>
        {
            _elapsedMin = (int)Math.Floor(v * 60.0 / 65536.0);
            RenderNav();
        });

        RegisterUInt("CLOCK_ELAPSED_TIME_SEC", v =>
        {
            _elapsedSec = (int)Math.Floor(v * 60.0 / 65536.0);
            RenderNav();
        });

        RegisterStr("EHSI_COURSE", s =>
        {
            _ehsiCourse = s.Trim();
            if (int.TryParse(s.Trim(), out int crs))
                _selectedCourseDeg = crs;
            RenderNav();
        });

        RegisterStr("EHSI_RANGE", s =>
        {
            string raw = s.Replace("\0", "").Trim();
            string normalized = NormalizeRangeText(raw);
            if (int.TryParse(normalized, out int rangeRaw))
                _ehsiRange = (rangeRaw / 10.0).ToString("F1", CultureInfo.InvariantCulture);
            else if (double.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out double rangeNm))
                _ehsiRange = rangeNm.ToString("F1", CultureInfo.InvariantCulture);
            else
                _ehsiRange = normalized;
            RenderNav();
        });

        RegisterStr("EHSI_MODE_LEFT",  s => { _ehsiModeLeft  = s.Trim(); RenderNav(); });
        RegisterStr("EHSI_MODE_RIGHT", s => { _ehsiModeRight = s.Trim(); RenderNav(); });
    }

    private void RenderNav()
    {
        var o = GetCompositor(NAV_PAGE);

        o.Line(0).Amber().WriteLine($"F-16C NAV  STDBY HDG:{_currentHeadingDeg:D3}");

        string rng = _ehsiRangeInvalid ? "---" : (_ehsiRange.Length > 0 ? _ehsiRange : "---");
        string crs = _ehsiCourse.Length > 0 ? _ehsiCourse : "---";
        o.Line(1).Green().WriteLine($"RNG:{rng,6}nm    CRS:{crs,3}");

        string modeL = _ehsiModeLeft.Length  > 0 ? _ehsiModeLeft  : "   ";
        string modeR = _ehsiModeRight.Length > 0 ? _ehsiModeRight : "NAV";
        o.Line(2).Green().WriteLine($"MODE:{modeL} {modeR,-18}");

        o.Line(3).Green().WriteLine(new string('-', 24));

        string vs = _vviFpm >= 0 ? $"+{_vviFpm,5}" : $"{_vviFpm,6}";
        o.Line(4).Green().WriteLine($"ALT:{_altFt,6}ft VS:{vs}");

        o.Line(5).Green().WriteLine($"IAS:{_iasKts,6}kt MCH:{_mach,4:F2}");

        string qnh  = $"{_qnhD3}{_qnhD2}.{_qnhD1}{_qnhD0}";
        string pneu = _pneuFail ? "FAIL" : " OK ";
        o.Line(6).Green().WriteLine($"QNH:{qnh,6}in PNU:{pneu}");

        o.Line(7).Green().WriteLine(new string('-', 24));

        o.Line(8).Green().WriteLine($"TOT:{_fuelTotalLb,5}lb FF:{_fuelFlowPph,5}");
        o.Line(9).Green().WriteLine($"AFT:{_fuelAlLb,5}lb FWD:{_fuelFrLb,4}");

        double endurH  = _fuelFlowPph > 0 ? (double)_fuelTotalLb / _fuelFlowPph : 0;
        int    endurMin = (int)(endurH * 60);
        int    rangeNm  = (int)(endurH * 300);
        o.Line(10).ClearRow().Green().Write($"END:{endurMin / 60,2}h{endurMin % 60,2}").Column(12).WriteLine($"RNG~{rangeNm,4}nm");

        o.Line(11).Green().WriteLine(new string('-', 24));
        o.Line(12).Green().WriteLine($"Clock:{_clockH:D2}:{_clockMin:D2}  HACK:{_elapsedMin:D2}:{_elapsedSec:D2}");

        if (_fuelLow)
            o.Line(13).Amber().WriteLine("** FUEL LOW **      (NAV)");
        else
            o.Line(13).Green().WriteLine($"{"(NAV)",24}");
    }

    private void RecomputeAltitude() =>
        _altFt = _altD10k * 10000 + _altD1k * 1000 + _altLowFt;

    private void RecomputeFuelTotal() =>
        _fuelTotalLb = _fuelTot10kDigit * 10000 + _fuelTot1kDigit * 1000 + _fuelTot100Digit * 100;

    private static int ComposeFuelFlowPph(int d10k, int d1k, int d100) =>
        d10k * 10000 + d1k * 1000 + d100;

    private static int KnobToDegrees(uint raw) =>
        (int)Math.Round(raw / 65535.0 * 359.0) % 360;

    private static int DecodeVviFpm(uint raw)
    {
        // DCS-BIOS reports the VVI gauge needle position, not ft/min.
        // Aircraft LUA: CreateGaugeLocal(16, {-10000, -6000, 0, 6000}, {-1.0, -0.77, 0.0, 1.0}).
        // The gauge is asymmetric, so the raw value must be mapped back per segment.
        double position = raw / 65535.0 * 2.0 - 1.0;

        if (position <= -0.77)
            return (int)Math.Round(Interpolate(position, -1.0, -0.77, -10000.0, -6000.0));

        if (position < 0.0)
            return (int)Math.Round(Interpolate(position, -0.77, 0.0, -6000.0, 0.0));

        return (int)Math.Round(Interpolate(position, 0.0, 1.0, 0.0, 6000.0));
    }

    private static double Interpolate(double value, double fromLow, double fromHigh, double toLow, double toHigh)
    {
        if (fromHigh == fromLow) return toLow;
        double t = (value - fromLow) / (fromHigh - fromLow);
        return toLow + t * (toHigh - toLow);
    }

    // Aircraft LUA: CreateGaugeLocal(49, {Mach...}, {needle...}, controllers.MachIndicator).
    // DCS-BIOS reports the needle position, which decreases as Mach increases.
    private static readonly double[] MachNeedlePositions =
    {
        1.0, 0.958, 0.921, 0.902, 0.885, 0.848, 0.812, 0.775, 0.741,
        0.704, 0.668, 0.632, 0.596, 0.562, 0.528, 0.493, 0.459, 0.422, 0.387
    };

    private static readonly double[] MachNumbers =
    {
        0.0, 0.5, 1.0, 1.05, 1.1, 1.2, 1.3, 1.4, 1.5,
        1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5
    };

    private static double DecodeMach(double needlePosition)
    {
        // Invert the asymmetric gauge mapping by locating the matching needle segment.
        if (needlePosition >= MachNeedlePositions[0]) return MachNumbers[0];

        int last = MachNeedlePositions.Length - 1;
        if (needlePosition <= MachNeedlePositions[last]) return MachNumbers[last];

        for (int i = 0; i < last; i++)
        {
            if (needlePosition <= MachNeedlePositions[i] && needlePosition >= MachNeedlePositions[i + 1])
            {
                return Interpolate(needlePosition,
                    MachNeedlePositions[i], MachNeedlePositions[i + 1],
                    MachNumbers[i],         MachNumbers[i + 1]);
            }
        }

        return MachNumbers[last];
    }

    // Aircraft LUA: CreateGaugeLocal(48, {knots...}, {needle...}, controllers.AirSpeedIndicator).
    // DCS-BIOS reports the needle position, which increases with airspeed.
    private static readonly double[] AirspeedNeedlePositions =
    {
        0.0, 0.042, 0.095, 0.152, 0.182, 0.199, 0.255, 0.303,
        0.355, 0.402, 0.455, 0.653, 0.698, 0.750, 0.797, 0.850
    };

    private static readonly double[] AirspeedKnots =
    {
        0.0, 80.0, 100.0, 150.0, 170.0, 200.0, 250.0, 300.0,
        350.0, 400.0, 450.0, 650.0, 700.0, 750.0, 800.0, 850.0
    };

    private static double DecodeAirspeed(double needlePosition)
    {
        // Invert the non-linear gauge mapping by locating the matching needle segment.
        if (needlePosition <= AirspeedNeedlePositions[0]) return AirspeedKnots[0];

        int last = AirspeedNeedlePositions.Length - 1;
        if (needlePosition >= AirspeedNeedlePositions[last]) return AirspeedKnots[last];

        for (int i = 0; i < last; i++)
        {
            if (needlePosition >= AirspeedNeedlePositions[i] && needlePosition <= AirspeedNeedlePositions[i + 1])
            {
                return Interpolate(needlePosition,
                    AirspeedNeedlePositions[i], AirspeedNeedlePositions[i + 1],
                    AirspeedKnots[i],            AirspeedKnots[i + 1]);
            }
        }

        return AirspeedKnots[last];
    }

    private static int DecodeDrumDigit(uint value, int maxValue)
    {
        if (maxValue == 0) return 0;
        double normalized = value / (double)maxValue;
        int digit = (int)Math.Floor(normalized * 10.0 + 0.05);
        return digit == 10 ? 0 : Math.Clamp(digit, 0, 9);
    }

    private static int DecodeSubThousandValue(uint value, int maxValue, int step)
    {
        if (maxValue == 0 || step <= 0) return 0;
        double scaled = value * 1000.0 / maxValue;
        int quantized = (int)Math.Round(scaled / step, MidpointRounding.AwayFromZero) * step;
        return Math.Clamp(quantized, 0, 1000 - step);
    }

    private static string NormalizeRangeText(string raw)
    {
        if (string.IsNullOrWhiteSpace(raw)) return string.Empty;
        string cleaned = raw.Replace(" ", "").Trim();
        if (cleaned.EndsWith("NM", StringComparison.OrdinalIgnoreCase))
            cleaned = cleaned[..^2];
        return cleaned.Trim();
    }
}
