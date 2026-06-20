using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class FA18C_Listener
{
    private string _fuelUp    = "      ";
    private string _fuelDown  = "      ";
    private string _bingo     = "     ";
    private string _ffL       = "   ";
    private string _ffR       = "   ";
    private string _rpmL      = "   ";
    private string _rpmR      = "   ";
    private string _tempL     = "   ";
    private string _tempR     = "   ";
    private string _oilPressL = "   ";
    private string _oilPressR = "   ";
    private string _clockH    = "  ";
    private string _clockM    = "  ";
    private string _clockS    = "  ";
    private string _timerH    = "  ";
    private string _timerM    = "  ";
    private string _timerS    = "  ";

    private void RegisterIfeiControls()
    {
        RegisterStr("IFEI_FUEL_UP",    s => { _fuelUp    = s; RenderIfei(); });
        RegisterStr("IFEI_FUEL_DOWN",  s => { _fuelDown  = s; RenderIfei(); });
        RegisterStr("IFEI_BINGO",      s => { _bingo     = s; RenderIfei(); });
        RegisterStr("IFEI_FF_L",       s => { _ffL       = s; RenderIfei(); });
        RegisterStr("IFEI_FF_R",       s => { _ffR       = s; RenderIfei(); });
        RegisterStr("IFEI_RPM_L",      s => { _rpmL      = s; RenderIfei(); });
        RegisterStr("IFEI_RPM_R",      s => { _rpmR      = s; RenderIfei(); });
        RegisterStr("IFEI_TEMP_L",     s => { _tempL     = s; RenderIfei(); });
        RegisterStr("IFEI_TEMP_R",     s => { _tempR     = s; RenderIfei(); });
        RegisterStr("IFEI_OIL_PRESS_L",s => { _oilPressL = s; RenderIfei(); });
        RegisterStr("IFEI_OIL_PRESS_R",s => { _oilPressR = s; RenderIfei(); });
        RegisterStr("IFEI_CLOCK_H",    s => { _clockH = s; RenderIfei(); });
        RegisterStr("IFEI_CLOCK_M",    s => { _clockM = s; RenderIfei(); });
        RegisterStr("IFEI_CLOCK_S",    s => { _clockS = s; RenderIfei(); });
        RegisterStr("IFEI_TIMER_H",    s => { _timerH = s; RenderIfei(); });
        RegisterStr("IFEI_TIMER_M",    s => { _timerM = s; RenderIfei(); });
        RegisterStr("IFEI_TIMER_S",    s => { _timerS = s; RenderIfei(); });
    }

    private void RegisterIfeiFrontPanelControls()
    {
        RegisterStr("IFEI_CLOCK_H", s => { _clockH = s; CombineIFEIClock(); });
        RegisterStr("IFEI_CLOCK_M", s => { _clockM = s; CombineIFEIClock(); });
        RegisterStr("IFEI_CLOCK_S", s => { _clockS = s; CombineIFEIClock(); });
        RegisterStr("IFEI_TIMER_H", s => { _timerH = s; CombineIFEITimer(); });
        RegisterStr("IFEI_TIMER_M", s => { _timerM = s; CombineIFEITimer(); });
        RegisterStr("IFEI_TIMER_S", s => { _timerS = s; CombineIFEITimer(); });
    }

    private void RenderIfei()
    {
        var output = GetCompositor(IFEI_PAGE);
        bool isDay = _lightMode == 0;

        if (isDay) output.White(); else output.Yellow();
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
        output.Line(13).WriteLine(string.Format("OIL            {0,3}   {1,3}", _oilPressL, _oilPressR));
    }

    private void CombineIFEIClock()
    {
        FlightDeck.ClockUtcTime = $"{_clockH}{_clockM}{_clockS}";
    }

    private void CombineIFEITimer()
    {
        FlightDeck.ClockElapsedTime = $"{_timerM}{_timerS}";
        if (_timerH != " 0" && _timerH != "  ")
            FlightDeck.ClockChrono = $"{_timerH}{_timerM}";
    }
}
