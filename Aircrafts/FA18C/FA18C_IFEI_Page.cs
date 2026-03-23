using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
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

    public void ProcessData(DCSBIOSStringDataEventArgs e)
    {
        if (_ifeiFuelUp != null && e.Address.Equals(_ifeiFuelUp.Address))
            _fuelUp = e.StringData;
        if (_ifeiFuelDown != null && e.Address.Equals(_ifeiFuelDown.Address))
            _fuelDown = e.StringData;
        if (_ifeiBingo != null && e.Address.Equals(_ifeiBingo.Address))
            _bingo = e.StringData;
        if (_ifeiFfL != null && e.Address.Equals(_ifeiFfL.Address))
            _ffL = e.StringData;
        if (_ifeiFfR != null && e.Address.Equals(_ifeiFfR.Address))
            _ffR = e.StringData;
        if (_ifeiRpmL != null && e.Address.Equals(_ifeiRpmL.Address))
            _rpmL = e.StringData;
        if (_ifeiRpmR != null && e.Address.Equals(_ifeiRpmR.Address))
            _rpmR = e.StringData;
        if (_ifeiTempL != null && e.Address.Equals(_ifeiTempL.Address))
            _tempL = e.StringData;
        if (_ifeiTempR != null && e.Address.Equals(_ifeiTempR.Address))
            _tempR = e.StringData;
        if (_ifeiOilPressL != null && e.Address.Equals(_ifeiOilPressL.Address))
            _oilPressL = e.StringData;
        if (_ifeiOilPressR != null && e.Address.Equals(_ifeiOilPressR.Address))
            _oilPressR = e.StringData;
        if (_ifeiClockH != null && e.Address.Equals(_ifeiClockH.Address))
            _clockH = e.StringData;
        if (_ifeiClockM != null && e.Address.Equals(_ifeiClockM.Address))
            _clockM = e.StringData;
        if (_ifeiClockS != null && e.Address.Equals(_ifeiClockS.Address))
            _clockS = e.StringData;
        if (_ifeiTimerH != null && e.Address.Equals(_ifeiTimerH.Address))
            _timerH = e.StringData;
        if (_ifeiTimerM != null && e.Address.Equals(_ifeiTimerM.Address))
            _timerM = e.StringData;
        if (_ifeiTimerS != null && e.Address.Equals(_ifeiTimerS.Address))
            _timerS = e.StringData;
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
}
