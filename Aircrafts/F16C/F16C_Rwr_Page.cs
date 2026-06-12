using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class F16C_Rwr_Page
{
    private DCSBIOSOutput? _LIGHT_RWR_POWER;
    private DCSBIOSOutput? _LIGHT_RWR_ACTIVITY;
    private DCSBIOSOutput? _LIGHT_RWR_SEARCH;
    private DCSBIOSOutput? _LIGHT_RWR_ALT;
    private DCSBIOSOutput? _LIGHT_RWR_ALT_LOW;
    private DCSBIOSOutput? _LIGHT_RWR_MSL_LAUNCH;
    private DCSBIOSOutput? _LIGHT_RWR_MODE_PRI;
    private DCSBIOSOutput? _LIGHT_RWR_MODE_OPEN;
    private DCSBIOSOutput? _LIGHT_RWR_HANDOFF_H;
    private DCSBIOSOutput? _LIGHT_RWR_SHIP_UNK;
    private DCSBIOSOutput? _LIGHT_RWR_TGTSEP_DN;
    private DCSBIOSOutput? _LIGHT_RWR_TGTSEP_UP;
    private DCSBIOSOutput? _LIGHT_RWR_SYSTEST;

    private DCSBIOSOutput? _LIGHT_ECM_1_A;
    private DCSBIOSOutput? _LIGHT_ECM_1_F;
    private DCSBIOSOutput? _LIGHT_ECM_1_S;
    private DCSBIOSOutput? _LIGHT_ECM_1_T;
    private DCSBIOSOutput? _LIGHT_ECM_2_A;
    private DCSBIOSOutput? _LIGHT_ECM_2_F;
    private DCSBIOSOutput? _LIGHT_ECM_2_S;
    private DCSBIOSOutput? _LIGHT_ECM_2_T;
    private DCSBIOSOutput? _LIGHT_ECM_3_A;
    private DCSBIOSOutput? _LIGHT_ECM_3_F;
    private DCSBIOSOutput? _LIGHT_ECM_3_S;
    private DCSBIOSOutput? _LIGHT_ECM_3_T;
    private DCSBIOSOutput? _LIGHT_ECM_4_A;
    private DCSBIOSOutput? _LIGHT_ECM_4_F;
    private DCSBIOSOutput? _LIGHT_ECM_4_S;
    private DCSBIOSOutput? _LIGHT_ECM_4_T;
    private DCSBIOSOutput? _LIGHT_ECM_5_A;
    private DCSBIOSOutput? _LIGHT_ECM_5_F;
    private DCSBIOSOutput? _LIGHT_ECM_5_S;
    private DCSBIOSOutput? _LIGHT_ECM_5_T;

    private DCSBIOSOutput? _CMDS_CH_AMOUNT;
    private DCSBIOSOutput? _CMDS_FL_AMOUNT;
    private DCSBIOSOutput? _CMDS_O1_AMOUNT;
    private DCSBIOSOutput? _CMDS_O2_AMOUNT;
    private DCSBIOSOutput? _LIGHT_CMDS_RDY;
    private DCSBIOSOutput? _LIGHT_CMDS_GO;
    private DCSBIOSOutput? _LIGHT_CMDS_DISP;

    private bool _rwrPower;
    private bool _rwrActivity;
    private bool _rwrSearch;
    private bool _rwrAlt;
    private bool _rwrAltLow;
    private bool _rwrMslLaunch;
    private bool _rwrModePri;
    private bool _rwrModeOpen;
    private bool _rwrHandoff;
    private bool _rwrShipUnk;
    private bool _rwrTgtSepDn;
    private bool _rwrTgtSepUp;
    private bool _rwrSysTest;

    private bool _ecm1A;
    private bool _ecm1F;
    private bool _ecm1S;
    private bool _ecm1T;
    private bool _ecm2A;
    private bool _ecm2F;
    private bool _ecm2S;
    private bool _ecm2T;
    private bool _ecm3A;
    private bool _ecm3F;
    private bool _ecm3S;
    private bool _ecm3T;
    private bool _ecm4A;
    private bool _ecm4F;
    private bool _ecm4S;
    private bool _ecm4T;
    private bool _ecm5A;
    private bool _ecm5F;
    private bool _ecm5S;
    private bool _ecm5T;

    private bool _cmdsRdy;
    private bool _cmdsGo;
    private bool _cmdsDisp;
    private string _cmdsCh = "---";
    private string _cmdsFl = "---";
    private string _cmdsO1 = "---";
    private string _cmdsO2 = "---";

    public void InitializeControls()
    {
        _LIGHT_RWR_POWER = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_POWER");
        _LIGHT_RWR_ACTIVITY = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ACTIVITY");
        _LIGHT_RWR_SEARCH = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SEARCH");
        _LIGHT_RWR_ALT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ALT");
        _LIGHT_RWR_ALT_LOW = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ALT_LOW");
        _LIGHT_RWR_MSL_LAUNCH = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MSL_LAUNCH");
        _LIGHT_RWR_MODE_PRI = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MODE_PRI");
        _LIGHT_RWR_MODE_OPEN = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MODE_OPEN");
        _LIGHT_RWR_HANDOFF_H = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_HANDOFF_H");
        _LIGHT_RWR_SHIP_UNK = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SHIP_UNK");
        _LIGHT_RWR_TGTSEP_DN = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_TGTSEP_DN");
        _LIGHT_RWR_TGTSEP_UP = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_TGTSEP_UP");
        _LIGHT_RWR_SYSTEST = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SYSTEST");

        _LIGHT_ECM_1_A = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_1_A");
        _LIGHT_ECM_1_F = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_1_F");
        _LIGHT_ECM_1_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_1_S");
        _LIGHT_ECM_1_T = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_1_T");
        _LIGHT_ECM_2_A = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_2_A");
        _LIGHT_ECM_2_F = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_2_F");
        _LIGHT_ECM_2_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_2_S");
        _LIGHT_ECM_2_T = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_2_T");
        _LIGHT_ECM_3_A = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_3_A");
        _LIGHT_ECM_3_F = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_3_F");
        _LIGHT_ECM_3_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_3_S");
        _LIGHT_ECM_3_T = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_3_T");
        _LIGHT_ECM_4_A = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_4_A");
        _LIGHT_ECM_4_F = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_4_F");
        _LIGHT_ECM_4_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_4_S");
        _LIGHT_ECM_4_T = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_4_T");
        _LIGHT_ECM_5_A = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_5_A");
        _LIGHT_ECM_5_F = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_5_F");
        _LIGHT_ECM_5_S = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_5_S");
        _LIGHT_ECM_5_T = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_ECM_5_T");

        _CMDS_CH_AMOUNT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMDS_CH_AMOUNT");
        _CMDS_FL_AMOUNT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMDS_FL_AMOUNT");
        _CMDS_O1_AMOUNT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMDS_O1_AMOUNT");
        _CMDS_O2_AMOUNT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("CMDS_O2_AMOUNT");
        _LIGHT_CMDS_RDY = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_RDY");
        _LIGHT_CMDS_GO = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_GO");
        _LIGHT_CMDS_DISP = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_DISP");
    }

    public bool ProcessData(DCSBIOSDataEventArgs e)
    {
        bool refreshDisplay = false;

        void L(DCSBIOSOutput? ctrl, ref bool field)
        {
            if (ctrl != null && ctrl.Address == e.Address)
            {
                field = ctrl.GetUIntValue(e.Data) == 1;
                refreshDisplay = true;
            }
        }

        L(_LIGHT_RWR_POWER, ref _rwrPower);
        L(_LIGHT_RWR_ACTIVITY, ref _rwrActivity);
        L(_LIGHT_RWR_SEARCH, ref _rwrSearch);
        L(_LIGHT_RWR_ALT, ref _rwrAlt);
        L(_LIGHT_RWR_ALT_LOW, ref _rwrAltLow);
        L(_LIGHT_RWR_MSL_LAUNCH, ref _rwrMslLaunch);
        L(_LIGHT_RWR_MODE_PRI, ref _rwrModePri);
        L(_LIGHT_RWR_MODE_OPEN, ref _rwrModeOpen);
        L(_LIGHT_RWR_HANDOFF_H, ref _rwrHandoff);
        L(_LIGHT_RWR_SHIP_UNK, ref _rwrShipUnk);
        L(_LIGHT_RWR_TGTSEP_DN, ref _rwrTgtSepDn);
        L(_LIGHT_RWR_TGTSEP_UP, ref _rwrTgtSepUp);
        L(_LIGHT_RWR_SYSTEST, ref _rwrSysTest);
        L(_LIGHT_ECM_1_A, ref _ecm1A); L(_LIGHT_ECM_1_F, ref _ecm1F);
        L(_LIGHT_ECM_1_S, ref _ecm1S); L(_LIGHT_ECM_1_T, ref _ecm1T);
        L(_LIGHT_ECM_2_A, ref _ecm2A); L(_LIGHT_ECM_2_F, ref _ecm2F);
        L(_LIGHT_ECM_2_S, ref _ecm2S); L(_LIGHT_ECM_2_T, ref _ecm2T);
        L(_LIGHT_ECM_3_A, ref _ecm3A); L(_LIGHT_ECM_3_F, ref _ecm3F);
        L(_LIGHT_ECM_3_S, ref _ecm3S); L(_LIGHT_ECM_3_T, ref _ecm3T);
        L(_LIGHT_ECM_4_A, ref _ecm4A); L(_LIGHT_ECM_4_F, ref _ecm4F);
        L(_LIGHT_ECM_4_S, ref _ecm4S); L(_LIGHT_ECM_4_T, ref _ecm4T);
        L(_LIGHT_ECM_5_A, ref _ecm5A); L(_LIGHT_ECM_5_F, ref _ecm5F);
        L(_LIGHT_ECM_5_S, ref _ecm5S); L(_LIGHT_ECM_5_T, ref _ecm5T);
        L(_LIGHT_CMDS_RDY, ref _cmdsRdy);
        L(_LIGHT_CMDS_GO, ref _cmdsGo);
        L(_LIGHT_CMDS_DISP, ref _cmdsDisp);

        return refreshDisplay;
    }

    public bool ProcessData(DCSBIOSStringDataEventArgs e)
    {
        bool rwrChanged = false;

        if (_CMDS_CH_AMOUNT != null && e.Address == _CMDS_CH_AMOUNT.Address)
        {
            _cmdsCh = e.StringData.Trim();
            rwrChanged = true;
        }

        if (_CMDS_FL_AMOUNT != null && e.Address == _CMDS_FL_AMOUNT.Address)
        {
            _cmdsFl = e.StringData.Trim();
            rwrChanged = true;
        }

        if (_CMDS_O1_AMOUNT != null && e.Address == _CMDS_O1_AMOUNT.Address)
        {
            _cmdsO1 = e.StringData.Trim();
            rwrChanged = true;
        }

        if (_CMDS_O2_AMOUNT != null && e.Address == _CMDS_O2_AMOUNT.Address)
        {
            _cmdsO2 = e.StringData.Trim();
            rwrChanged = true;
        }

        return rwrChanged;
    }

    public void Render(Compositor o)
    {
        static string Ind(bool on) => on ? "ON " : "-- ";

        o.Line(0).Amber().WriteLine("F-16C RWR / EW STATUS   ");
        o.Line(1).Green().WriteLine(new string('-', 24));

        string mode = _rwrModePri ? "PRI" : _rwrModeOpen ? "OPN" : "---";
        o.Line(2).Green().Write("PWR:");
        if (_rwrPower) o.Amber().Write("ON "); else o.Green().Write("-- ");
        o.Green().Write("  MODE:");
        if (_rwrModePri || _rwrModeOpen) o.Amber().Write(mode); else o.Green().Write(mode);
        o.Green().WriteLine("  ");

        o.Line(3).Green().Write("ACT:");
        if (_rwrActivity) o.Amber().Write("YES"); else o.Green().Write("-- ");
        o.Green().Write("  SRCH:");
        if (_rwrSearch) o.Amber().Write("YES"); else o.Green().Write("-- ");
        o.Green().WriteLine("    ");

        o.Line(4).Green().WriteLine(new string('-', 24));

        if (_rwrMslLaunch)
        {
            o.Line(5).Black().BGAmber().WriteLine("  *** MSL LAUNCH ***    ");
        }
        else
        {
            o.Line(5).Green().WriteLine("  ( no launch detected )");
        }

        o.Line(6).Green().Write("ALT:");
        if (_rwrAlt) o.Amber().Write("ACT "); else o.Green().Write("--- ");
        o.Green().Write("ALTLOW:");
        if (_rwrAltLow) o.Amber().Write("YES"); else o.Green().Write("-- ");
        o.Green().WriteLine(" ");

        o.Line(7).Green()
            .Write($"HND:{Ind(_rwrHandoff)} ")
            .Write($"UNK:{Ind(_rwrShipUnk)} ")
            .Write($"SEP:{(_rwrTgtSepDn ? "DN" : _rwrTgtSepUp ? "UP" : "--")}");

        o.Line(8).Green().WriteLine(new string('-', 24));
        o.Line(9).Green().WriteLine("ECM  A  F  S  T         ");

        var pods = new (string Name, bool A, bool F, bool S, bool T)[]
        {
            ("POD1", _ecm1A, _ecm1F, _ecm1S, _ecm1T),
            ("POD2", _ecm2A, _ecm2F, _ecm2S, _ecm2T),
            ("POD3", _ecm3A, _ecm3F, _ecm3S, _ecm3T),
            ("POD4", _ecm4A, _ecm4F, _ecm4S, _ecm4T),
            ("POD5", _ecm5A, _ecm5F, _ecm5S, _ecm5T),
        };

        for (int i = 0; i < pods.Length && i + 10 < Metrics.Lines; i++)
        {
            var (name, a, f, s, t) = pods[i];
            o.Line(10 + i).Green().Write($"{name} ");
            if (a) o.Amber().Write("A"); else o.Green().Write(".");
            o.Green().Write("  ");
            if (f) o.Amber().Write("F"); else o.Green().Write(".");
            o.Green().Write("  ");
            if (s) o.Amber().Write("S"); else o.Green().Write(".");
            o.Green().Write("  ");
            if (t) o.Amber().Write("T"); else o.Green().Write(".");
            o.Green().WriteLine("         ");
        }

        o.Line(Metrics.Lines - 1).Green()
            .Write($"CH:{_cmdsCh,3} FL:{_cmdsFl,3} ")
            .Amber().Write("(RWR)");
    }
}
