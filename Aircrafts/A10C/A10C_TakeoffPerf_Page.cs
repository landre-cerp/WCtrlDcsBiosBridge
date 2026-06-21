using System.Globalization;
using a10c_perf_lib.src;
using WwDevicesDotNet;
using static a10c_perf_lib.src.PerfCalculator;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// A-10C Takeoff Performance CDU page.
/// <para>
/// The pilot enters gross weight (GW) directly — easiest read off the DCS loadout/refuel
/// page — and picks taxi fuel (100/200/300 lb). Takeoff weight TOW = GW − taxi fuel, and all
/// performance is computed against that.
/// </para>
/// <para>
/// QNH is taken silently from the aircraft altimeter
/// (<see cref="FlightDeckState.BaroPressure"/>), falling back to standard 29.92 inHg.
/// Flaps fixed at 7° (<see cref="FLAPS.TO"/>). Computes PTFS, rotate/takeoff speed, ground
/// run and critical field length via A10C-PerfLib; ground run and CFL turn red when they
/// exceed the entered runway length.
/// </para>
/// Rendering only ever happens on the input thread (key/scratchpad).
/// </summary>
internal partial class A10C_Listener
{
    // Scratchpad-committed inputs (raw strings; parsed on compute).
    private string? _elevFt;
    private string? _rwyLenFt;
    private string? _qfuDeg;
    private string? _wind;        // combined "DIR/SPD", e.g. "200/15"
    private string? _tempC;
    private string? _gw;          // gross weight entered manually (boxed)
    private RCR _rcr = RCR.DRY;   // cycled via LSK; feeds critical field length
    private int _taxiFuel = 100;  // taxi fuel 100/200/300 lb, cycled like RCR; subtracted from GW

    // Computed outputs (null = not enough/invalid input to compute).
    private double? _tow;
    private double? _ptfs;
    private double? _rotate;
    private double? _takeoff;
    private double? _groundRun;
    private double? _cfl;
    private TakeoffIndex? _toIndex;
    private bool _rwyTooShort;
    private bool _cflTooLong;
    private string? _error;

    private void RegisterTakeoffControls()
    {
        Scratchpad.Changed += (_, _) =>
        {
            if (_currentPage == TAKEOFF_PAGE)
                RenderTakeoffPage();
        };
    }

    private void HandleTakeoffKey(Key key)
    {
        switch (key)
        {
            case Key.LineSelectLeft1: Scratchpad.CommitToField(ref _elevFt); break;
            case Key.LineSelectLeft2: Scratchpad.CommitToField(ref _rwyLenFt); break;
            case Key.LineSelectLeft3: Scratchpad.CommitToField(ref _qfuDeg); break;
            case Key.LineSelectLeft4: Scratchpad.CommitToField(ref _wind); break;
            case Key.LineSelectLeft5: Scratchpad.CommitToField(ref _gw); break;
            case Key.LineSelectRight1: Scratchpad.CommitToField(ref _tempC); break;
            case Key.LineSelectRight3: CycleRcr(); break;
            case Key.LineSelectRight5: CycleTaxi(); break;

            default:
                // FuelPred toggles the page; nav keys also leave. Back to the live A-10C CDU.
                if (key == _nextPageKey || key == _prevPageKey || key == Key.FuelPred)
                {
                    Scratchpad.Clear();
                    _currentPage = DEFAULT_PAGE; // refreshed by the display timer + DCS-BIOS handlers
                }
                return;
        }

        Compute();
        RenderTakeoffPage();
    }

    private void CycleRcr() =>
        _rcr = _rcr switch { RCR.DRY => RCR.WET, RCR.WET => RCR.ICY, _ => RCR.DRY };

    private void CycleTaxi() =>
        _taxiFuel = _taxiFuel switch { 100 => 200, 200 => 300, _ => 100 };

    private void Compute()
    {
        _error = null;
        _ptfs = _rotate = _takeoff = _groundRun = _cfl = null;
        _toIndex = null;
        _rwyTooShort = _cflTooLong = false;

        double? temp = ParseD(_tempC);
        double? elev = ParseD(_elevFt);
        double? rwy = ParseD(_rwyLenFt);
        double? qfu = ParseD(_qfuDeg);
        double? gw = ParseD(_gw);
        (double? windDir, double? windSpd) = ParseWind(_wind);

        // Takeoff weight = entered gross weight − taxi fuel burned before takeoff.
        _tow = gw.HasValue ? gw.Value - _taxiFuel : (double?)null;

        // QNH masked: use the aircraft altimeter when available (inHg ×100), else standard.
        QNH qnh = FlightDeck.BaroPressure is int baro && baro > 0
            ? new InHgQNH(baro / 100.0)
            : QNH.StdInHg;

        try
        {
            if (temp.HasValue)
                _ptfs = PTFS(temp.Value);

            if (temp.HasValue && elev.HasValue)
                _toIndex = CalcTakeoffIndex(temp.Value, new PressureAltitude(elev.Value, qnh));

            if (_tow is double tow)
            {
                _takeoff = TakeOffSpeed(tow, FLAPS.TO);
                _rotate = RotationSpeed(tow, FLAPS.TO);

                if (_toIndex.HasValue)
                {
                    double headwind = 0;
                    if (qfu.HasValue && windDir.HasValue && windSpd.HasValue)
                        headwind = windSpd.Value * Math.Cos((windDir.Value - qfu.Value) * Math.PI / 180.0);

                    _groundRun = TakeoffGroundRun(_toIndex.Value, (GrossWeight)tow, headwind, FLAPS.TO);
                    _cfl = CriticalFieldLength(_toIndex.Value, (GrossWeight)tow, headwind, FLAPS.TO, _rcr);

                    if (rwy.HasValue)
                    {
                        _rwyTooShort = _groundRun > rwy.Value;
                        _cflTooLong = _cfl > rwy.Value;
                    }
                }
            }
        }
        catch (ArgumentOutOfRangeException ex)
        {
            _error = ex.ParamName switch
            {
                "Grossweight" or "pounds" or "grossWeight" => "GW RANGE",
                "temperature" => "TEMP RANGE",
                _ => "INPUT RANGE",
            };
        }
    }

    private void RenderTakeoffPage()
    {
        var c = GetCompositor(TAKEOFF_PAGE);
        c.Clear();

        c.Line(0).White().Centered("A-10C TKOFF");
        c.Column(Metrics.Columns - 3).White().Write("FL7");

        // Units sit next to the label: right of left-side labels, left of right-side labels.
        // Values carry no units.
        LabelLeft(c, 1, "ELEV(ft)");     BoxLeft(c, 2, _elevFt, 5);     // L1
        LabelRight(c, 1, "(°C)TEMP");    BoxRight(c, 2, _tempC, 4);     // R1

        // RWY length + required ground run (green = fits / red = too short);
        // CFL = critical field length, red when it exceeds the runway.
        LabelLeft(c, 3, "RWY LEN(ft)");  LabelRight(c, 3, "(ft)CFL");
        BoxLeft(c, 4, _rwyLenFt, 5);                                     // L2
        RenderGroundRunInline(c);
        ColoredRight(c, 4, _cfl is double cfl ? cfl.ToString("0") : "----",
                     _cfl.HasValue ? _cflTooLong : (bool?)null);

        LabelLeft(c, 5, "QFU(°)");       BoxLeft(c, 6, _qfuDeg, 3);     // L3
        LabelRight(c, 5, "RCR");          CycleRight(c, 6, _rcr.ToString()); // R3 (cycle)

        LabelLeft(c, 7, "WIND(°/kt)");   BoxLeft(c, 8, _wind, 7);       // L4
        LabelRight(c, 7, "PTFS");         PlainRight(c, 8, Fmt(_ptfs, "0")); // moved up

        // Gross weight (boxed input, L5) + takeoff weight (middle) + taxi fuel (cycle, R5).
        LabelLeft(c, 9, "GW ");
        c.Line(9).Column(9).Small().White().Write("TOW");
        LabelRight(c, 9, "(lbs)TAXI");

        BoxLeft(c, 10, _gw, 6);                                          // L5
        c.Line(10).Column(9).Large().Green().Write(Fmt(_tow, "0"));      // TOW middle
        CycleRight(c, 10, _taxiFuel.ToString());                         // R5 (cycle)

        // Rotate left, takeoff speed right-aligned.
        c.Line(11).Small().White().Write("VR(kt) ").Large().Green().Write(Fmt(_rotate, "0"));
        string toLabel = "(kt)TO ";
        string tov = Fmt(_takeoff, "0");
        c.Column(Metrics.Columns - (toLabel.Length + tov.Length))
         .Small().White().Write(toLabel).Large().Green().Write(tov);

        if (_error != null)
            c.Line(12).Large().Amber().Write(_error);

        // --- Scratchpad ---
        c.Line(13).Cyan().Write(Scratchpad.DisplayText);
    }

    // Writes " GR <groundRun>" after the RWY box: green if it fits the runway, red if not.
    private void RenderGroundRunInline(Compositor c)
    {
        c.Small().White().Write(" GR ");
        if (_groundRun is double gr)
        {
            if (_rwyTooShort) c.Large().Red();
            else c.Large().Green();
            c.Write(gr.ToString("0", CultureInfo.InvariantCulture));
        }
        else
        {
            c.Large().White().Write("----");
        }
    }

    private static void LabelLeft(Compositor c, int line, string label) =>
        c.Line(line).Small().White().Write(label);

    private static void LabelRight(Compositor c, int line, string label) =>
        c.Line(line).Column(Metrics.Columns - label.Length).Small().White().Write(label);

    private static void BoxLeft(Compositor c, int line, string? value, int width) =>
        c.Line(line).Large().Cyan().Write(BoxField(value, width));

    private static void BoxRight(Compositor c, int line, string? value, int width) =>
        c.Line(line).Column(Metrics.Columns - width).Large().Cyan().Write(BoxField(value, width));

    private static void PlainRight(Compositor c, int line, string value) =>
        c.Line(line).Column(Metrics.Columns - value.Length).Large().Green().Write(value);

    // Right-aligned cyclable value in the input colour (cyan), tagged with Δ to mark it as
    // a toggle (e.g. "DRY Δ", "200 Δ"). Only calculated read-outs are green.
    private static void CycleRight(Compositor c, int line, string value)
    {
        string text = value + " Δ";
        c.Line(line).Column(Metrics.Columns - text.Length).Large().Cyan().Write(text);
    }

    // Right-aligned large value: red when bad==true, green when bad==false, white when bad==null.
    private static void ColoredRight(Compositor c, int line, string text, bool? bad)
    {
        c.Line(line).Column(Metrics.Columns - text.Length).Large();
        if (bad == true) c.Red();
        else if (bad == false) c.Green();
        else c.White();
        c.Write(text);
    }

    private static string Fmt(double? value, string format) =>
        value.HasValue ? value.Value.ToString(format, CultureInfo.InvariantCulture) : "---";

    private static double? ParseD(string? s) =>
        s != null && double.TryParse(s, NumberStyles.Any, CultureInfo.InvariantCulture, out var v)
            ? v
            : null;

    // Parse a combined "DIR/SPD" wind entry (e.g. "200/15") into direction + speed.
    private static (double? dir, double? spd) ParseWind(string? s)
    {
        if (s == null) return (null, null);
        var parts = s.Split('/');
        return parts.Length == 2 ? (ParseD(parts[0]), ParseD(parts[1])) : (null, null);
    }
}
