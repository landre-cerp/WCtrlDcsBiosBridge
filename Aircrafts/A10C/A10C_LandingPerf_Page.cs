using System.Globalization;
using a10c_perf_lib.src;
using WwDevicesDotNet;
using static a10c_perf_lib.src.PerfCalculator;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class A10C_Listener
{
    // Inputs — all independent from the takeoff page fields.
    private string? _lndElevFt;
    private string? _lndTempC;
    private string? _lndQfuDeg;
    private string? _lndWind;
    private string? _lndRwyLenFt;
    private string? _lndGw = EMPTY_WEIGHT_LBS.ToString("0");  // pre-filled; user overrides with actual landing weight
    private RCR _lndRcr = RCR.DRY;
    private Speedbrakes _lndSpeedbrakes = Speedbrakes.Open;
    private bool _lndSingleEngine = false;
    private bool _lndMinSpeed = false;

    // Computed outputs (null = not enough/invalid input to compute).
    private double? _lndApproachSpeed;
    private double? _lndTouchdownSpeed;
    private double? _lndGroundRoll;
    private bool _lndRwyTooShort;
    private string? _lndError;

    private void RegisterLandingControls()
    {
        Scratchpad.Changed += (_, _) =>
        {
            if (_currentPage == LANDING_PAGE)
                RenderLandingPage();
        };
    }

    private void HandleLandingKey(Key key)
    {
        switch (key)
        {
            case Key.LineSelectLeft1: Scratchpad.CommitToField(ref _lndElevFt); break;
            case Key.LineSelectLeft2: Scratchpad.CommitToField(ref _lndQfuDeg); break;
            case Key.LineSelectLeft3: Scratchpad.CommitToField(ref _lndWind); break;
            case Key.LineSelectLeft4: Scratchpad.CommitToField(ref _lndRwyLenFt); break;
            case Key.LineSelectLeft5: Scratchpad.CommitToField(ref _lndGw); break;
            case Key.LineSelectRight1: Scratchpad.CommitToField(ref _lndTempC); break;
            case Key.LineSelectRight2: CycleLndRcr(); break;
            case Key.LineSelectRight3: _lndSingleEngine = !_lndSingleEngine; break;
            case Key.LineSelectRight4: CycleLndSpeedbrakes(); break;
            case Key.LineSelectRight5: _lndMinSpeed = !_lndMinSpeed; break;

            default:
                if (key == _nextPageKey || key == _prevPageKey)
                {
                    Scratchpad.Clear();
                    _currentPage = TAKEOFF_PAGE;
                    Compute();
                    RenderTakeoffPage();
                }
                else if (key == _perfPageKey)
                {
                    Scratchpad.Clear();
                    _currentPage = DEFAULT_PAGE;
                }
                return;
        }

        ComputeLanding();
        RenderLandingPage();
    }

    private void CycleLndRcr() =>
        _lndRcr = _lndRcr switch { RCR.DRY => RCR.WET, RCR.WET => RCR.ICY, _ => RCR.DRY };

    private void CycleLndSpeedbrakes() =>
        _lndSpeedbrakes = _lndSpeedbrakes == Speedbrakes.Open ? Speedbrakes.Closed : Speedbrakes.Open;

    private void ComputeLanding()
    {
        _lndError = null;
        _lndApproachSpeed = _lndTouchdownSpeed = _lndGroundRoll = null;
        _lndRwyTooShort = false;

        double? lndGw = ParseD(_lndGw);
        double? temp = ParseD(_lndTempC);
        double? elev = ParseD(_lndElevFt);
        double? rwy = ParseD(_lndRwyLenFt);
        double? qfu = ParseD(_lndQfuDeg);
        (double? windDir, double? windSpd) = ParseWind(_lndWind);

        QNH qnh = FlightDeck.BaroPressure is int baro && baro > 0
            ? new InHgQNH(baro / 100.0)
            : QNH.StdInHg;

        try
        {
            if (lndGw.HasValue)
            {
                var gw = new GrossWeight(lndGw.Value);
                _lndApproachSpeed = ApproachSpeed(gw, LandingFlaps.TWENTY, _lndSingleEngine, _lndMinSpeed);
                _lndTouchdownSpeed = TouchdownSpeed(gw, LandingFlaps.TWENTY, _lndSingleEngine, _lndMinSpeed);

                if (temp.HasValue && elev.HasValue)
                {
                    var landingIndex = CalcLandingIndex(temp.Value, new PressureAltitude(elev.Value, qnh));

                    double headwind = 0;
                    if (qfu.HasValue && windDir.HasValue && windSpd.HasValue)
                        headwind = windSpd.Value * Math.Cos((windDir.Value - qfu.Value) * Math.PI / 180.0);

                    _lndGroundRoll = LandingGroundRoll(landingIndex, gw, _lndSpeedbrakes, headwind, _lndRcr, _lndMinSpeed);

                    if (rwy.HasValue && _lndGroundRoll.HasValue)
                        _lndRwyTooShort = _lndGroundRoll.Value > rwy.Value;
                }
            }
        }
        catch (ArgumentOutOfRangeException ex)
        {
            _lndError = ex.ParamName switch
            {
                "Grossweight" or "pounds" or "grossWeight" => "GW RANGE",
                "temperature" => "TEMP RANGE",
                _ => "INPUT RANGE",
            };
        }
    }

    private void RenderLandingPage()
    {
        var c = GetCompositor(LANDING_PAGE);
        c.Clear();

        c.Line(0).Small().White().Write("2/2").Centered("LANDING");
        c.Column(Metrics.Columns - 4).White().Write("FL20");

        LabelLeft(c, 1, "ELEV(ft)");    BoxLeft(c, 2, _lndElevFt, 5);      // L1
        LabelRight(c, 1, "(°C)TEMP");   BoxRight(c, 2, _lndTempC, 4);      // R1

        LabelLeft(c, 3, "QFU(°)");      BoxLeft(c, 4, _lndQfuDeg, 3);      // L2
        LabelRight(c, 3, "RCR");        CycleRight(c, 4, _lndRcr.ToString()); // R2

        LabelLeft(c, 5, "WIND(°/kt)");  BoxLeft(c, 6, _lndWind, 7);        // L3
        LabelRight(c, 5, "1 ENG.");     CycleRight(c, 6, _lndSingleEngine ? "YES" : "NO"); // R3

        LabelLeft(c, 7, "RWY(ft)");     LabelRight(c, 7, "SPD BRK");
        BoxLeft(c, 8, _lndRwyLenFt, 5);                                     // L4
        RenderLandingGroundRunInline(c);
        CycleRight(c, 8, _lndSpeedbrakes == Speedbrakes.Open ? "OPEN" : "CLSD"); // R4

        LabelLeft(c, 9, "GW(lbs)");          LabelRight(c, 9, "MIN SPD");
        BoxLeft(c, 10, _lndGw, 6);                                          // L5
        CycleRight(c, 10, _lndMinSpeed ? "YES" : "NO");                     // R5

        // Speed outputs: VAPP left, VTD right — mirrors VR/TO layout on takeoff page.
        c.Line(11).Small().White().Write("VAPP(kt) ").Large().Green().Write(Fmt(_lndApproachSpeed, "0"));
        string tdLabel = "(kt)VTD ";
        string tdv = Fmt(_lndTouchdownSpeed, "0");
        c.Column(Metrics.Columns - (tdLabel.Length + tdv.Length))
         .Small().White().Write(tdLabel).Large().Green().Write(tdv);

        if (_lndError != null)
            c.Line(12).Large().Amber().Write(_lndError);

        c.Line(13).Cyan().Write(Scratchpad.DisplayText);
    }

    private void RenderLandingGroundRunInline(Compositor c)
    {
        c.Small().White().Write(" GR ");
        if (_lndGroundRoll is double gr)
        {
            if (_lndRwyTooShort) c.Large().Red();
            else c.Large().Green();
            c.Write(gr.ToString("0", CultureInfo.InvariantCulture));
        }
        else
        {
            c.Large().White().Write("----");
        }
    }
}
