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

    // IFF (AN/APX-76) — all on RIO panel
    private int _iffMaster  = 0;  // 0=OFF 1=STBY 2=LOW 3=NORM 4=EMER
    private int _iffM1      = 0;  // 0=OFF 1=ON 2=TEST
    private int _iffM1_10   = 0;  // M1 code 10s digit
    private int _iffM1_1    = 0;  // M1 code 1s digit
    private int _iffM2      = 0;  // 0=OFF 1=ON 2=TEST
    private int _iffM3a     = 0;  // 0=OFF 1=ON 2=TEST
    private int _iffM3_1000 = 0;  // M3 code 1000s digit
    private int _iffM3_100  = 0;  // M3 code 100s digit
    private int _iffM3_10   = 0;  // M3 code 10s digit
    private int _iffM3_1    = 0;  // M3 code 1s digit
    private int _iffM4      = 0;  // 0=OFF 1=ON
    private int _iffIdent   = 0;  // 0=OFF 1=MIC 2=IDENT
    private int _iffCode    = 0;  // 0=HLD 1=A 2=B 3=ZRO

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

        RegisterUInt("RIO_IFF_MASTER",  v => { _iffMaster  = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M1",      v => { _iffM1      = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M1_10",   v => { _iffM1_10   = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M1_1",    v => { _iffM1_1    = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M2",      v => { _iffM2      = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M3A",     v => { _iffM3a     = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M3_1000", v => { _iffM3_1000 = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M3_100",  v => { _iffM3_100  = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M3_10",   v => { _iffM3_10   = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M3_1",    v => { _iffM3_1    = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_M4",      v => { _iffM4      = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_IDENT",   v => { _iffIdent   = (int)v; RenderRadioPage(); });
        RegisterUInt("RIO_IFF_CODE",    v => { _iffCode    = (int)v; RenderRadioPage(); });

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

    // IFF AN/APX-76 label arrays (standard panel positions)
    private static readonly string[] _iffMasterLabels = { "OFF", "STBY", "LOW", "NORM", "EMER" };
    private static readonly string[] _iffModeLabels   = { "OFF", "ON ", "TST" };
    private static readonly string[] _iffIdentLabels  = { "OFF", "MIC", "IDT" };
    private static readonly string[] _iffCodeLabels   = { "HLD", "A  ", "B  ", "ZRO" };

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

        // IFF AN/APX-76 section — lines 5-10
        c.White().Line(5).Write(new string('-', Metrics.Columns));

        string iffMaster = _iffMaster < _iffMasterLabels.Length ? _iffMasterLabels[_iffMaster] : "?";
        // "IFF  MASTER:{0,-4}        " = 12 + 4 + 8 = 24
        c.White().Line(6).WriteLine(string.Format("IFF  MASTER:{0,-4}        ", iffMaster));

        string m1  = _iffM1  < _iffModeLabels.Length ? _iffModeLabels[_iffM1]  : "?  ";
        string m2  = _iffM2  < _iffModeLabels.Length ? _iffModeLabels[_iffM2]  : "?  ";
        string m3a = _iffM3a < _iffModeLabels.Length ? _iffModeLabels[_iffM3a] : "?  ";
        // "M1:xxx  M2:xxx  M3:xxx  " = 3+3+2+3+3+2+3+3+2 = 24
        c.White().Line(7).WriteLine(string.Format("M1:{0}  M2:{1}  M3:{2}  ", m1, m2, m3a));

        string m4    = _iffM4    == 1 ? "ON " : "OFF";
        string ident = _iffIdent < _iffIdentLabels.Length ? _iffIdentLabels[_iffIdent] : "?  ";
        string code  = _iffCode  < _iffCodeLabels.Length  ? _iffCodeLabels[_iffCode]   : "?  ";
        // "M4:xxx  IDT:xxx  COD:xxx" = 3+3+2+4+3+2+4+3 = 24
        c.White().Line(8).WriteLine(string.Format("M4:{0}  IDT:{1}  COD:{2}", m4, ident, code));

        // "  M1:XX        M3:XXXX  " = 5+2+8+3+4+2 = 24
        c.Cyan().Line(9).WriteLine(string.Format("  M1:{0}{1}        M3:{2}{3}{4}{5}  ",
            _iffM1_10, _iffM1_1, _iffM3_1000, _iffM3_100, _iffM3_10, _iffM3_1));

        c.White().Line(10).Write(new string(' ', Metrics.Columns));

        c.Line(Metrics.Lines - 1).BGBlack().White()
            .Write(new string(' ', Metrics.Columns - 7))
            .Amber().Write("(RADIO)");
    }
}
