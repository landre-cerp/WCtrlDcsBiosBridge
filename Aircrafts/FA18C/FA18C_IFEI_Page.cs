using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class FA18C_IFEI_Page
{
    private DCSBIOSOutput? _ifeiFuelUp;
    private DCSBIOSOutput? _ifeiFuelDown;
    private DCSBIOSOutput? _ifeiBingo;
    private DCSBIOSOutput? _ifeiFfL;
    private DCSBIOSOutput? _ifeiFfR;
    private DCSBIOSOutput? _ifeiRpmL;
    private DCSBIOSOutput? _ifeiRpmR;
    private DCSBIOSOutput? _ifeiTempL;
    private DCSBIOSOutput? _ifeiTempR;
    private DCSBIOSOutput? _ifeiOilPressL;
    private DCSBIOSOutput? _ifeiOilPressR;
    private DCSBIOSOutput? _ifeiClockH;
    private DCSBIOSOutput? _ifeiClockM;
    private DCSBIOSOutput? _ifeiClockS;
    private DCSBIOSOutput? _ifeiTimerH;
    private DCSBIOSOutput? _ifeiTimerM;
    private DCSBIOSOutput? _ifeiTimerS;

    string _fuelUp = "      ";
    string _fuelDown = "      ";
    string _bingo = "     ";
    string _ffL = "   ";
    string _ffR = "   ";
    string _rpmL = "   ";
    string _rpmR = "   ";
    string _tempL = "   ";
    string _tempR = "   ";
    string _oilPressL = "   ";
    string _oilPressR = "   ";
    string _clockH = "  ";
    string _clockM = "  ";
    string _clockS = "  ";
    string _timerH = "  ";
    string _timerM = "  ";
    string _timerS = "  ";


    public void InitializeControls()
    {
        _ifeiFuelUp = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_FUEL_UP");
        _ifeiFuelDown = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_FUEL_DOWN");
        _ifeiBingo = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_BINGO");
        _ifeiFfL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_FF_L");
        _ifeiFfR = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_FF_R");
        _ifeiRpmL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_RPM_L");
        _ifeiRpmR = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_RPM_R");
        _ifeiTempL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_TEMP_L");
        _ifeiTempR = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_TEMP_R");
        _ifeiOilPressL = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_OIL_PRESS_L");
        _ifeiOilPressR = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_OIL_PRESS_R");
        _ifeiClockH = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_CLOCK_H");
        _ifeiClockM = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_CLOCK_M");
        _ifeiClockS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_CLOCK_S");
        _ifeiTimerH = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_TIMER_H");
        _ifeiTimerM = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_TIMER_M");
        _ifeiTimerS = DCSBIOSControlLocator.GetStringDCSBIOSOutput("IFEI_TIMER_S");
    }

    public void RegisterControls(Action<DCSBIOSOutput?, Action<string>> registerString, Action onUpdated)
    {
        registerString(_ifeiFuelUp, s => { _fuelUp = s; onUpdated(); });
        registerString(_ifeiFuelDown, s => { _fuelDown = s; onUpdated(); });
        registerString(_ifeiBingo, s => { _bingo = s; onUpdated(); });
        registerString(_ifeiFfL, s => { _ffL = s; onUpdated(); });
        registerString(_ifeiFfR, s => { _ffR = s; onUpdated(); });
        registerString(_ifeiRpmL, s => { _rpmL = s; onUpdated(); });
        registerString(_ifeiRpmR, s => { _rpmR = s; onUpdated(); });
        registerString(_ifeiTempL, s => { _tempL = s; onUpdated(); });
        registerString(_ifeiTempR, s => { _tempR = s; onUpdated(); });
        registerString(_ifeiOilPressL, s => { _oilPressL = s; onUpdated(); });
        registerString(_ifeiOilPressR, s => { _oilPressR = s; onUpdated(); });
        registerString(_ifeiClockH, s => { _clockH = s; onUpdated(); });
        registerString(_ifeiClockM, s => { _clockM = s; onUpdated(); });
        registerString(_ifeiClockS, s => { _clockS = s; onUpdated(); });
        registerString(_ifeiTimerH, s => { _timerH = s; onUpdated(); });
        registerString(_ifeiTimerM, s => { _timerM = s; onUpdated(); });
        registerString(_ifeiTimerS, s => { _timerS = s; onUpdated(); });
    }

    public void Render(Compositor output, uint lightMode)
    {
        //  CDU layout: 24 chars wide, 14 rows (0-13)
        //
        //  Row 0:  "      F/A-18C IFEI   2/2"  (title and page number)
        //  Row 1:  "                        "
        //  Row 2:  "                  xxxxxx"  (fuel up)
        //  Row 3:  "                  xxxxxx"  (fuel down)
        //  Row 4:  "                   xxxxx"  (bingo)
        //  Row 5:  "                        "
        //  Row 6:  "       xx:xx:xx         "  (clock)
        //  Row 7:  "       xx:xx:xx         "  (timer)
        //  Row 8:  "                        "
        //  Row 9:  "              LEFT RIGHT"
        //  Row 10: "RPM            xxx   xxx"
        //  Row 11: "TEMP           xxx   xxx"
        //  Row 12: "FF             xxx   xxx"
        //  Row 13: "OIL            xxx   xxx"

        bool isDay = lightMode == 0;

        if (isDay)
            output.White();
        else
            output.Yellow();
        output.Line(0).Centered("F/A-18C IFEI").Column(21).Write("2/2");

        output.Line(1).ClearRow();

        output.Line(2).WriteLine(string.Format("{0,24}", _fuelUp));
        output.Line(3).WriteLine(string.Format("{0,24}", _fuelDown));
        output.Line(4).WriteLine(string.Format("{0,24}", _bingo));

        output.Line(5).ClearRow();

        output.Line(6).Centered(string.Format("{0}:{1}:{2}", _clockH, _clockM, _clockS));
        if (string.IsNullOrWhiteSpace(_timerH) && string.IsNullOrWhiteSpace(_timerM) && string.IsNullOrWhiteSpace(_timerS))
            output.Line(7).ClearRow();
        else
            output.Line(7).Centered(string.Format("{0}:{1}:{2}", _timerH, _timerM, _timerS));

        output.Line(8).ClearRow();

        output.Line(9).WriteLine("              LEFT  RIGHT");

        output.Line(10).WriteLine(string.Format("RPM            {0,3}   {1,3}", _rpmL, _rpmR));
        output.Line(11).WriteLine(string.Format("TEMP           {0,3}   {1,3}", _tempL, _tempR));
        output.Line(12).WriteLine(string.Format("FF             {0,3}   {1,3}", _ffL, _ffR));
        output
            .Line(13)
            .WriteLine(string.Format("OIL            {0,3}   {1,3}", _oilPressL, _oilPressR));
    }


    internal void RegisterFrontPanelControls(Action<DCSBIOSOutput?, Action<string>> registerString, FlightDeckState flightDeck)
    {
        registerString(_ifeiClockH, s => { _clockH = s; combineClock(flightDeck); });
        registerString(_ifeiClockM, s => { _clockM = s; combineClock(flightDeck); });
        registerString(_ifeiClockS, s => { _clockS = s; combineClock(flightDeck); });

        registerString(_ifeiTimerH, s => { _timerH = s; combineTimer(flightDeck); });
        registerString(_ifeiTimerM, s => { _timerM = s; combineTimer(flightDeck); });
        registerString(_ifeiTimerS, s => { _timerS = s; combineTimer(flightDeck); });
    }

    internal void combineClock(FlightDeckState flightDeck)
    {
        flightDeck.Agp32UtcTime = $"{_clockH}{_clockM}{_clockS}";

    }

    internal void combineTimer(FlightDeckState flightDeck)
    {

        flightDeck.Agp32Et = $"{_timerM}{_timerS}";
        if (_timerH != " 0" && _timerH != "  ")
        {
            flightDeck.Agp32Chrono = $"{_timerH}{_timerM}";
        }
        

    }

}
