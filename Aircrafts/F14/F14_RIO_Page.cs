namespace WCtrlDcsBiosBridge.Aircrafts.F14;

internal partial class F14_Listener
{
    private int _capPage;
    private int _currentHCUMode;
    private int _currentWCSmode;

    private static readonly string[] _hcuModes = { "TID", "TCS", "RADAR", "DDD" };
    private static readonly string[] _wcsModes = { "TWS Auto", "Puls Sch", "TWS Man", "PD Sch", "PD STT", "RWS", "Puls Stt" };
    private static readonly string[] _categoryNames = { "BIT", "SPL", "NAV", "TAC DATA", "D/L", "TGT DATA" };

    // Each CAP page is two string[5] arrays: even indices are the upper lines, odd are the lower lines.
    // Layout: _categoryLabelPages[2*page] = upper lines, _categoryLabelPages[2*page+1] = lower lines.
    private static readonly string[][] _categoryLabelPages =
    {
        // BIT
        new[] { "-   disp        rcvr   -", "-   cmptr       xmtr   -", "- amcs conf    ant ir  -", "- mas moat      stt    -", "-   fault     spl test -" },
        new[] { "      1          5      ", "      2          6      ", "      3          7      ", "      4          8      ", "    disp        nbr     " },
        // SPL
        new[] { "-   home                ", "-    ift        bit    -", "-   ip          obc    -", "-   gss         maint  -", "-   air to      obc    -" },
        new[] { "  on heli               ", "    menu    moving tgt  ", "  to tgt        bit     ", "                disp    ", "    ground     displ    " },
        // NAV
        new[] { "-   own        tacan   -", "- store hdg     rdr    -", "- tarps nav     vis    -", "-   wind        fix    -", "-   tarps     mag var  -" },
        new[] { "    a/c         fix     ", "    align       fix     ", "     fix        fix     ", "  spd hdg      enable   ", "               (hdg)    " },
        // TAC DATA
        new[] { "-  waypt        home   -", "-  waypt         def   -", "-  waypt        host   -", "-   fix         surf   -", "-    ip        pt to   -" },
        new[] { "     1          base    ", "     2           pt     ", "     3          area    ", "     pt         tgt     ", "                 pt     " },
        // D/L
        new[] { "-  wilco       point   -", "-  cantco      engage  -", "-   nav        flrp    -", "-   tid        chaff   -", "- f/f nav    f/f auto  -" },
        new[] { "                        ", "                        ", "    grid                ", "    avia       count    ", "  update       rstt     " },
        // TGT DATA
        new[] { "-   gnd        friend  -", "-  do not        unk   -", "-  ift aux      host   -", "-   data        mult   -", "-   test         sym   -" },
        new[] { "    map                 ", "   attack               ", "   launch               ", "    trans        tgt    ", "    tgt       delete    " },
    };

    private void RegisterRioControls()
    {
        // Note: "RIO_CAP_CATRGORY" is the actual DCS-BIOS control id (typo preserved from DCS-BIOS JSON)
        RegisterUInt("RIO_CAP_CATRGORY", v => { _capPage = (int)v; RenderRioPage(); });

        RegisterUInt("RIO_HCU_TID",   v => { if (v == 1) { _currentHCUMode = 0; RenderRioPage(); } });
        RegisterUInt("RIO_HCU_TCS",   v => { if (v == 1) { _currentHCUMode = 1; RenderRioPage(); } });
        RegisterUInt("RIO_HCU_RADAR", v => { if (v == 1) { _currentHCUMode = 2; RenderRioPage(); } });
        RegisterUInt("RIO_HCU_DDD",   v => { if (v == 1) { _currentHCUMode = 3; RenderRioPage(); } });

        RegisterUInt("RIO_RADAR_TWSAUTO", v => { if (v == 1) { _currentWCSmode = 0; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_PULSE",   v => { if (v == 1) { _currentWCSmode = 1; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_TWSMAN",  v => { if (v == 1) { _currentWCSmode = 2; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_PDSRCH",  v => { if (v == 1) { _currentWCSmode = 3; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_PDSTT",   v => { if (v == 1) { _currentWCSmode = 4; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_RWS",     v => { if (v == 1) { _currentWCSmode = 5; RenderRioPage(); } });
        RegisterUInt("RIO_RADAR_PSTT",    v => { if (v == 1) { _currentWCSmode = 6; RenderRioPage(); } });

        RenderRioPage();
    }

    private void RenderRioPage()
    {
        var c = GetCompositor(DEFAULT_PAGE);

        c.White().Line(0).WriteLine(
            string.Format("{0,-5} {1,-9}{2,10}",
                _hcuModes[_currentHCUMode],
                _wcsModes[_currentWCSmode],
                _categoryNames[_capPage]));

        int currentLine = 2;
        for (int line = 0; line < 5; line++)
        {
            c.White().Line(currentLine++).WriteLine(
                string.Format("{0,24}", _categoryLabelPages[2 * _capPage][line]));
            c.Cyan().Line(currentLine++).WriteLine(
                string.Format("{0,24}", _categoryLabelPages[2 * _capPage + 1][line]));
        }
    }
}
