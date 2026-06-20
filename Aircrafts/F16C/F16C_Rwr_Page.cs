using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class F16C_Listener
{
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

    private bool _ecm1A; private bool _ecm1F; private bool _ecm1S; private bool _ecm1T;
    private bool _ecm2A; private bool _ecm2F; private bool _ecm2S; private bool _ecm2T;
    private bool _ecm3A; private bool _ecm3F; private bool _ecm3S; private bool _ecm3T;
    private bool _ecm4A; private bool _ecm4F; private bool _ecm4S; private bool _ecm4T;
    private bool _ecm5A; private bool _ecm5F; private bool _ecm5S; private bool _ecm5T;

    private bool _cmdsRdy;
    private bool _cmdsGo;
    private bool _cmdsDisp;
    private string _cmdsCh = "---";
    private string _cmdsFl = "---";
    private string _cmdsO1 = "---";
    private string _cmdsO2 = "---";

    private void RegisterRwrControls()
    {
        void Light(string id, Action<bool> set)
            => RegisterUInt(id, v => { set(v == 1); RenderRwr(); });

        void Amount(string id, Action<string> set)
            => RegisterStr(id, s => { set(s.Trim()); RenderRwr(); });

        Light("LIGHT_RWR_POWER",      b => _rwrPower = b);
        Light("LIGHT_RWR_ACTIVITY",   b => _rwrActivity = b);
        Light("LIGHT_RWR_SEARCH",     b => _rwrSearch = b);
        Light("LIGHT_RWR_ALT",        b => _rwrAlt = b);
        Light("LIGHT_RWR_ALT_LOW",    b => _rwrAltLow = b);
        Light("LIGHT_RWR_MSL_LAUNCH", b => _rwrMslLaunch = b);
        Light("LIGHT_RWR_MODE_PRI",   b => _rwrModePri = b);
        Light("LIGHT_RWR_MODE_OPEN",  b => _rwrModeOpen = b);
        Light("LIGHT_RWR_HANDOFF_H",  b => _rwrHandoff = b);
        Light("LIGHT_RWR_SHIP_UNK",   b => _rwrShipUnk = b);
        Light("LIGHT_RWR_TGTSEP_DN",  b => _rwrTgtSepDn = b);
        Light("LIGHT_RWR_TGTSEP_UP",  b => _rwrTgtSepUp = b);
        Light("LIGHT_RWR_SYSTEST",    b => _rwrSysTest = b);

        Light("LIGHT_ECM_1_A", b => _ecm1A = b); Light("LIGHT_ECM_1_F", b => _ecm1F = b);
        Light("LIGHT_ECM_1_S", b => _ecm1S = b); Light("LIGHT_ECM_1_T", b => _ecm1T = b);
        Light("LIGHT_ECM_2_A", b => _ecm2A = b); Light("LIGHT_ECM_2_F", b => _ecm2F = b);
        Light("LIGHT_ECM_2_S", b => _ecm2S = b); Light("LIGHT_ECM_2_T", b => _ecm2T = b);
        Light("LIGHT_ECM_3_A", b => _ecm3A = b); Light("LIGHT_ECM_3_F", b => _ecm3F = b);
        Light("LIGHT_ECM_3_S", b => _ecm3S = b); Light("LIGHT_ECM_3_T", b => _ecm3T = b);
        Light("LIGHT_ECM_4_A", b => _ecm4A = b); Light("LIGHT_ECM_4_F", b => _ecm4F = b);
        Light("LIGHT_ECM_4_S", b => _ecm4S = b); Light("LIGHT_ECM_4_T", b => _ecm4T = b);
        Light("LIGHT_ECM_5_A", b => _ecm5A = b); Light("LIGHT_ECM_5_F", b => _ecm5F = b);
        Light("LIGHT_ECM_5_S", b => _ecm5S = b); Light("LIGHT_ECM_5_T", b => _ecm5T = b);

        Light("LIGHT_CMDS_RDY",  b => _cmdsRdy = b);
        Light("LIGHT_CMDS_GO",   b => _cmdsGo = b);
        Light("LIGHT_CMDS_DISP", b => _cmdsDisp = b);

        Amount("CMDS_CH_AMOUNT", s => _cmdsCh = s);
        Amount("CMDS_FL_AMOUNT", s => _cmdsFl = s);
        Amount("CMDS_O1_AMOUNT", s => _cmdsO1 = s);
        Amount("CMDS_O2_AMOUNT", s => _cmdsO2 = s);
    }

    private void RenderRwr()
    {
        static string Ind(bool on) => on ? "ON " : "-- ";

        var o = GetCompositor(RWR_PAGE);

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
            o.Line(5).Black().BGAmber().WriteLine("  *** MSL LAUNCH ***    ");
        else
            o.Line(5).Green().WriteLine("  ( no launch detected )");

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
