using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.F14;

internal partial class F14_Listener
{
    private string _uhfFreq        = "-------";
    private string _pltUhfDisp     = "";
    private int    _uhfFunction    = 1;  // DCS-BIOS bug: reversed → 0=OFF 1=MAIN 2=BOTH 3=ADF
    private int    _uhfFreqMode    = 1;  // DCS-BIOS bug: reversed → 0=PRESET 1=MANUAL 2=GUARD
    private int    _uhfSquelch     = 0;

    private string _vuhfFreq       = "-------";
    private string _rioVuhfDisp    = "";
    private int    _vuhfRadioMode  = 1;  // 0=OFF 1=T/R 2=T/R&G 3=DF 4=TEST
    private int    _vuhfFreqMode   = 1;  // 0=243 1=MAN 2=G 3=PRESET 4=READ 5=LOAD
    private int    _vuhfFmAm       = 1;  // 0=FM 1=AM
    private int    _vuhfSquelch    = 0;

    private void RegisterRadioControls()
    {
        RegisterStr("UHF_FREQ",          s => { _uhfFreq     = s.TrimEnd(); RenderRadioPage(); });
        RegisterStr("PLT_UHF_REMOTE_DISP", s => { _pltUhfDisp = s;          RenderRadioPage(); });
        RegisterStr("VUHF_FREQ",         s => { _vuhfFreq    = s.TrimEnd(); RenderRadioPage(); });
        RegisterStr("RIO_VUHF_DISP",     s => { _rioVuhfDisp = s;          RenderRadioPage(); });

        RegisterUInt("PLT_UHF1_FUNCTION",  v => { _uhfFunction  = (int)v; RenderRadioPage(); });
        RegisterUInt("PLT_UHF1_FREQ_MODE", v => { _uhfFreqMode  = (int)v; RenderRadioPage(); });
        RegisterUInt("PLT_UHF1_SQUELCH",   v => { _uhfSquelch   = (int)v; RenderRadioPage(); });

        RegisterUInt("RIO_VUHF_MODE",      v => { _vuhfRadioMode = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_VUHF_FREQ_MODE", v => { _vuhfFreqMode  = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_VUHF_FM_AM",     v => { _vuhfFmAm      = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_VUHF_SQUELCH",   v => { _vuhfSquelch   = (int)v; RenderRadioPage(); });

        RenderRadioPage();
    }

    private string UhfChannelLabel()
    {
        // DCS-BIOS bug: PLT_UHF1_FREQ_MODE is reversed — PRESET reports as 0, GUARD as 2
        if (_uhfFreqMode == 0) // PRESET
            return int.TryParse(_pltUhfDisp.Trim(), out int ch) ? $"CH{ch:D2}" : "CH??";

        return _uhfFreqMode switch
        {
            2 => "GRD", // GUARD
            _ => "MAN"
        };
    }

    private string VuhfChannelLabel()
    {
        if (_vuhfFreqMode == 3) // PRESET — read channel from RIO display
            return int.TryParse(_rioVuhfDisp.Trim(), out int ch) ? $"CH{ch:D2}" : "CH??";

        return _vuhfFreqMode switch
        {
            0 => "243",
            2 => "GRD",
            4 => "RDG",
            5 => "LDG",
            _ => "MAN"
        };
    }

    // DCS-BIOS bug: PLT_UHF1_FUNCTION and PLT_UHF1_FREQ_MODE report values in reverse
    // order compared to the JSON positions array. Labels below reflect the actual
    // DCS-BIOS output, not the JSON declaration.
    private static readonly string[] _uhfFunctionLabels   = { "OFF", "MAIN", "BOTH", "ADF" };
    private static readonly string[] _uhfFreqModeLabels   = { "PRESET", "MANUAL", "GUARD" };
    private static readonly string[] _vuhfRadioModeLabels = { "OFF", "T/R", "T/R&G", "DF", "TEST" };
    private static readonly string[] _vuhfFreqModeLabels  = { "243", "MAN", "G", "PRESET", "READ", "LOAD" };

    private void RenderRadioPage()
    {
        var c = GetCompositor(RADIO_PAGE);

        string uhfFunc     = _uhfFunction    < _uhfFunctionLabels.Length   ? _uhfFunctionLabels[_uhfFunction]    : "?";
        string uhfFreqMode = _uhfFreqMode    < _uhfFreqModeLabels.Length   ? _uhfFreqModeLabels[_uhfFreqMode]   : "?";
        string vuhfRadio   = _vuhfRadioMode  < _vuhfRadioModeLabels.Length ? _vuhfRadioModeLabels[_vuhfRadioMode] : "?";
        string vuhfFreqMode = _vuhfFreqMode  < _vuhfFreqModeLabels.Length  ? _vuhfFreqModeLabels[_vuhfFreqMode]  : "?";

        // UHF ARC-159 (pilot) — header: "UHF  {func,-5}    {freqMode,-10}" = 5+5+4+10 = 24
        c.White().Line(0).WriteLine(string.Format("UHF  {0,-5}    {1,-10}", uhfFunc, uhfFreqMode));
        if (_uhfFunction == 0) // OFF (DCS-BIOS bug: reversed, OFF=0)
            c.Cyan().Line(1).WriteLine(string.Format("  {0,-22}", "OFF"));
        else
            c.Cyan().Line(1).WriteLine(string.Format("  {0,-4}  {1,-7}      {2,-3}", UhfChannelLabel(), _uhfFreq, _uhfSquelch == 1 ? "SQL" : ""));

        c.White().Line(2).Write(new string(' ', Metrics.Columns));

        // VUHF ARC-182 (RIO) — header: "VUHF {mode,-5}    {freqMode,-10}" = 5+5+4+10 = 24
        c.White().Line(3).WriteLine(string.Format("VUHF {0,-5}    {1,-10}", vuhfRadio, vuhfFreqMode));
        if (_vuhfRadioMode == 0) // OFF
            c.Cyan().Line(4).WriteLine(string.Format("  {0,-22}", "OFF"));
        else
            c.Cyan().Line(4).WriteLine(string.Format("  {0,-4}  {1,-7}  {2}  {3,-3}",
                VuhfChannelLabel(), _vuhfFreq, _vuhfFmAm == 0 ? "FM" : "AM", _vuhfSquelch == 1 ? "SQL" : ""));

        for (int r = 5; r < Metrics.Lines - 1; r++)
            c.White().Line(r).Write(new string(' ', Metrics.Columns));

        c.Line(Metrics.Lines - 1).BGBlack().White()
            .Write(new string(' ', Metrics.Columns - 7))
            .Amber().Write("(RADIO)");
    }
}
