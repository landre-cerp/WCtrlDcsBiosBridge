using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;
using WWCduDcsBiosBridge.Frontpanels;

namespace WWCduDcsBiosBridge.Aircrafts;

// ─────────────────────────────────────────────────────────────────────────────
// Single-screen switching pattern  (v1.0 — DED and HSI)
//
// The WinCtrl MCDU has one physical screen.  Two logical views — DED and HSI —
// are selected by the configured prev/next-page keys:
//
//   PrevPage key  (default LeftArrow)  → DED
//   NextPage key  (default RightArrow) → HSI
//
// Only the active view processes incoming BIOS events; the idle view burns
// zero CPU.  MFD views are reserved for v1.1+.
// ─────────────────────────────────────────────────────────────────────────────
internal enum DisplayMode { DED, NAV, RWR }

internal class F16C_Listener : AircraftListener
{
    // =========================================================================
    // Active display state
    // =========================================================================
    private volatile DisplayMode _currentDisplay = DisplayMode.DED;

    // =========================================================================
    // Console / Shared outputs
    // =========================================================================

    // PRI_CONSOLES_BRT_KNB  0x440E  UInt 0–65535  Primary console backlight
    private DCSBIOSOutput? _PRI_CONSOLES_BRT;

    // LIGHT_MASTER_CAUTION  0x447A  UInt 0/1  Cockpit lamp (read this, not 0x4400 button input)
    private DCSBIOSOutput? _LIGHT_MASTER_CAUTION;

    // =========================================================================
    // DED — Data Entry Display
    // =========================================================================
    //
    // Physical DED: 5 rows × 24 columns, monochrome green phosphor.
    // MCDU screen:  14 rows × 24 columns  (Metrics.Lines × Metrics.Columns).
    //
    // BIOS exports three parallel string arrays per line:
    //   DED_LINE_N     — full 24-char row as visible on the DED
    //   DED_LN         — same row, but non-inverse positions replaced with spaces
    //   DED_LN_FORMAT  — per-char mask: 'i' = inverse video, ' ' = normal
    //
    // Rendering strategy:
    //   Row i (0-4) of the MCDU shows DED_LINE_{i+1}.
    //   Characters where FORMAT[pos]=='i' are drawn with Black-on-BGGreen
    //   (true inverse video simulation on the MCDU LCD).
    //   Rows 5–13 are cleared to avoid ghost data from other display modes.
    // =========================================================================

    // DED brightness knob — used to drive MCDU display brightness
    // PRI_DATA_DISPLAY_BRT_KNB  0x4412  UInt 0–65535
    private DCSBIOSOutput? _PRI_DATA_DISPLAY_BRT;

    // Main lines: indices 0–4 → DED_LINE_1..DED_LINE_5
    // Addresses: 0x450A / 0x4528 / 0x4546 / 0x4564 / 0x4582
    private readonly DCSBIOSOutput?[] _dedMainLines = new DCSBIOSOutput?[5];

    // Secondary lines: indices 0–4 → DED_L1..DED_L5  (inverse chars, rest = spaces)
    // Addresses: 0x45EE / 0x4606 / 0x461E / 0x4636 / 0x464E
    private readonly DCSBIOSOutput?[] _dedSecLines = new DCSBIOSOutput?[5];

    // Format masks: indices 0–4 → DED_L1_FORMAT..DED_L5_FORMAT
    // 'i' at position N = inverse video; ' ' at position N = normal
    // Addresses: 0x4666 / 0x467E / 0x4696 / 0x46AE / 0x46C6
    private readonly DCSBIOSOutput?[] _dedFormats = new DCSBIOSOutput?[5];

    // Live cache — updated by DCSBIOSStringReceived, consumed by UpdateDEDDisplay
    private readonly string[] _dedMainText   = new string[5];
    private readonly string[] _dedFormatText = new string[5];

    // Dirty-check shadow — only rows whose (text, format) pair has changed get repainted
    private readonly string[] _dedRenderedMain = new string[5];
    private readonly string[] _dedRenderedFmt  = new string[5];

    // ─── F-16C DED character substitution table ───────────────────────────────
    //
    // DCS-BIOS emits raw ISO-8859-1 bytes for glyph positions the 7-segment DED
    // hardware renders as custom symbols.  Map them to the closest printable char.
    //
    // To extend: run DCS with logging enabled and capture any byte values that
    // appear in DED_LINE_* strings that don't display correctly on the MCDU.
    private static readonly Dictionary<char, char> DedCharMap = new()
    {
        { '\x00', ' '  },   // Null / trailing padding  → space
        { '\x01', '*'  },   // Filled-star glyph        → asterisk (ICP cursor marker)
        { '\x02', '>'  },   // Right-arrow segment      → '>'
        { '\x03', '<'  },   // Left-arrow segment       → '<'
        { '\x04', '^'  },   // Up-arrow segment         → '^'
        { '\x05', 'v'  },   // Down-arrow segment       → 'v'
        { '\x06', '-'  },   // Em-dash segment          → '-'
        { '\x08', ' '  },   // Backspace glyph          → space
        { '\x0B', '['  },   // Left-bracket segment
        { '\x0C', ']'  },   // Right-bracket segment
        { '\x0E', '+'  },   // Plus / cross segment
        { '\x0F', '='  },   // Double-bar segment
        { '\x10', '*'  },   // Open-star variant        → asterisk
        { '\x11', 'T'  },   // TACAN T-marker
        { '\x12', 'I'  },   // ILS marker
        { '\x14', '%'  },   // Percent sign
        { '\x16', '_'  },   // Underscore segment
        { '©',    '^'  },   // Latin-1 compat (A-10C)   → caret
        { '®',    'D'  },   // Latin-1 compat           → Delta (closest monospace)
        { '¡',    '↑'  },   // DCS up-arrow glyph
        { 'i',    '↓'  },   // DCS down-arrow glyph
        { '±',    '_'  },   // Latin-1 compat           → underscore
        { '«',    '<'  },   // Latin-1 compat           → '<'
        { '»',    '>'  },   // Latin-1 compat           → '>'
        { '°',    '°'  },   // Degree sign              → '°'
        { 'a',    '↕'  },   // stacked double-arrow (STPT selector)
    };

    // =========================================================================
    // MFD1 — Left Multi-Function Display
    //
    // DCS-BIOS exports NO framebuffer for F-16C MFDs.  Only switch/brightness
    // states are available as outputs.  The MFD display modes on the MCDU show
    // derived BIOS data (master mode, SMS page, etc.) that you wire up separately.
    // =========================================================================
    private DCSBIOSOutput? _MFD_L_BRT_SW;   // 0x4462  UInt 0–2  brightness rocker

    // =========================================================================
    // MFD2 — Right Multi-Function Display  (same limitation as MFD1)
    // =========================================================================
    private DCSBIOSOutput? _MFD_R_BRT_SW;   // 0x4464  UInt 0–2  brightness rocker

    // =========================================================================
    // Display-switching key bindings
    // =========================================================================
    // Parsed from UserOptions at construction time.
    //   _dedDisplayKey  (default PrevPage)   → DED
    //   _navDisplayKey  (default NextPage)   → NAV
    //   _rwrDisplayKey  (default F1)         → RWR
    private readonly Key _dedDisplayKey;
    private readonly Key _navDisplayKey;
    private readonly Key _rwrDisplayKey;

    // =========================================================================
    // HSI — Enhanced Horizontal Situation Indicator (EHSI)
    // =========================================================================

    // EHSI text outputs
    private DCSBIOSOutput? _EHSI_COURSE;      // 0x45E6  String  e.g. "270"
    private DCSBIOSOutput? _EHSI_RANGE;       // 0x45EA  String  e.g. "12.3"
    private DCSBIOSOutput? _EHSI_MODE_LEFT;   // 0x45DE  String  blank or "PLS"
    private DCSBIOSOutput? _EHSI_MODE_RIGHT;  // 0x45E2  String  "NAV", "TCN", etc.

    // EHSI integer outputs
    private DCSBIOSOutput? _EHSI_RANGE_INVALID; // 0x4580  UInt 0/1  strikethrough flag
    private DCSBIOSOutput? _EHSI_CRS_SET_KNB;   // 0x4470  UInt 0–65535  course knob
    private DCSBIOSOutput? _EHSI_HDG_SET_KNB;   // 0x4472  UInt 0–65535  heading bug knob

    private DCSBIOSOutput? _STANDBY_COMPASS_HEADING;

    // EHSI cached values
    private string _ehsiCourse      = "";
    private string _ehsiRange       = "";
    private string _ehsiModeLeft    = "";
    private string _ehsiModeRight   = "";
    private bool   _ehsiRangeInvalid;
    private int    _ehsiCrsDeg;
    private int    _ehsiHdgDeg;

    private int  _currentHeadingDeg  = 0;
    private int  _selectedCourseDeg  = 0;
    private int  _hdgBugDeg          = 0;
    private uint _lastHdgKnobValue   = 0;
    private bool _rangeInvalid       = false;

    // NAV page — altitude
    private DCSBIOSOutput? _ALT_10000;   // 0x4496
    private DCSBIOSOutput? _ALT_1000;    // 0x4498
    private DCSBIOSOutput? _ALT_100;     // 0x449A
    private DCSBIOSOutput? _ALT_PNEU;   // 0x44A4

    // NAV page — QNH drums
    private DCSBIOSOutput? _QNH_D0;     // 0x449C
    private DCSBIOSOutput? _QNH_D1;     // 0x449E
    private DCSBIOSOutput? _QNH_D2;     // 0x44A0
    private DCSBIOSOutput? _QNH_D3;     // 0x44A2

    // NAV page — airspeed / VVI
    private DCSBIOSOutput? _AIRSPEED;   // 0x44A6
    private DCSBIOSOutput? _MACH;       // 0x44AC
    private DCSBIOSOutput? _VVI;        // 0x44B8

    // NAV page — fuel totalizer
    private DCSBIOSOutput? _FUEL_TOT_10K; // 0x44EE
    private DCSBIOSOutput? _FUEL_TOT_1K;  // 0x44F0
    private DCSBIOSOutput? _FUEL_TOT_100; // 0x44F2

    // NAV page — fuel flow
    private DCSBIOSOutput? _FUEL_FF_10K;  // 0x44E4
    private DCSBIOSOutput? _FUEL_FF_1K;   // 0x44E6
    private DCSBIOSOutput? _FUEL_FF_100;  // 0x44E8

    // NAV page — tank gauges and warnings
    private DCSBIOSOutput? _FUEL_AL;          // 0x44EA
    private DCSBIOSOutput? _FUEL_FR;          // 0x44EC
    private DCSBIOSOutput? _LIGHT_FUEL_LOW;   // 0x4478

    // NAV page — clock
    private DCSBIOSOutput? _CLOCK_H;          // 0x4502
    private DCSBIOSOutput? _CLOCK_MS;         // 0x4504
    private DCSBIOSOutput? _CLOCK_ELAPSED_M;  // 0x4506
    private DCSBIOSOutput? _CLOCK_ELAPSED_S;  // 0x4508

    // NAV page — cached values
    private int  _altFt;
    private int  _altD10k = 0;
    private int  _altD1k  = 0;
    private int  _altD100 = 0;
    private int  _iasKts;
    private double _mach;
    private int  _vviFpm;
    private int  _fuelTotalLb;
    private int  _fuelFlowPph;
    private int  _fuelAlLb;
    private int  _fuelFrLb;
    private bool _fuelLow;
    private bool _pneuFail;
    private int  _qnhD0, _qnhD1, _qnhD2, _qnhD3;
    private int  _clockH, _clockMin;
    private int  _elapsedMin, _elapsedSec;

    // ── RWR indicator lights ───────────────────────────────────────────────
    private DCSBIOSOutput? _LIGHT_RWR_POWER;       // 0x447E
    private DCSBIOSOutput? _LIGHT_RWR_ACTIVITY;    // 0x447E
    private DCSBIOSOutput? _LIGHT_RWR_SEARCH;      // 0x447E
    private DCSBIOSOutput? _LIGHT_RWR_ALT;         // 0x447E
    private DCSBIOSOutput? _LIGHT_RWR_ALT_LOW;     // 0x447E
    private DCSBIOSOutput? _LIGHT_RWR_MSL_LAUNCH;  // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_MODE_PRI;    // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_MODE_OPEN;   // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_HANDOFF_H;   // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_SHIP_UNK;    // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_TGTSEP_DN;   // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_TGTSEP_UP;   // 0x4480
    private DCSBIOSOutput? _LIGHT_RWR_SYSTEST;     // 0x4480

    // ── ECM pod lights (5 pods × 4 states) ────────────────────────────────
    private DCSBIOSOutput? _LIGHT_ECM_1_A; // 0x4480
    private DCSBIOSOutput? _LIGHT_ECM_1_F; // 0x4480
    private DCSBIOSOutput? _LIGHT_ECM_1_S; // 0x4480
    private DCSBIOSOutput? _LIGHT_ECM_1_T; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_2_A; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_2_F; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_2_S; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_2_T; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_3_A; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_3_F; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_3_S; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_3_T; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_4_A; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_4_F; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_4_S; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_4_T; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_5_A; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_5_F; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_5_S; // 0x448A
    private DCSBIOSOutput? _LIGHT_ECM_5_T; // 0x448C

    // ── CMDS countermeasures string displays ───────────────────────────────
    private DCSBIOSOutput? _CMDS_CH_AMOUNT; // 0x45A8  string
    private DCSBIOSOutput? _CMDS_FL_AMOUNT; // 0x45AC  string
    private DCSBIOSOutput? _CMDS_O1_AMOUNT; // 0x45A0  string
    private DCSBIOSOutput? _CMDS_O2_AMOUNT; // 0x45A4  string
    private DCSBIOSOutput? _LIGHT_CMDS_RDY;  // 0x4480
    private DCSBIOSOutput? _LIGHT_CMDS_GO;   // 0x4480
    private DCSBIOSOutput? _LIGHT_CMDS_DISP; // 0x4480

    // ── RWR/ECM/CMDS cached booleans ──────────────────────────────────────
    private bool _rwrPower, _rwrActivity, _rwrSearch;
    private bool _rwrAlt, _rwrAltLow, _rwrMslLaunch;
    private bool _rwrModePri, _rwrModeOpen;
    private bool _rwrHandoff, _rwrShipUnk;
    private bool _rwrTgtSepDn, _rwrTgtSepUp, _rwrSysTest;
    private bool _ecm1A, _ecm1F, _ecm1S, _ecm1T;
    private bool _ecm2A, _ecm2F, _ecm2S, _ecm2T;
    private bool _ecm3A, _ecm3F, _ecm3S, _ecm3T;
    private bool _ecm4A, _ecm4F, _ecm4S, _ecm4T;
    private bool _ecm5A, _ecm5F, _ecm5S, _ecm5T;
    private bool _cmdsRdy, _cmdsGo, _cmdsDisp;
    private string _cmdsCh = "---", _cmdsFl = "---";
    private string _cmdsO1 = "---", _cmdsO2 = "---";
    private bool _rwrLaunchBlink;

    // =========================================================================
    // Constructor / destructor
    // =========================================================================
    protected override string GetAircraftName() => SupportedAircrafts.F16C_Name;
    protected override string GetFontFile()     => "resources/f16c-font-21x31.json";

    public F16C_Listener(
        ICdu? mcdu,
        UserOptions options,
        FrontpanelHub frontpanelHub) : base(mcdu, SupportedAircrafts.F16C, options, frontpanelHub)
    {
        for (int i = 0; i < 5; i++)
        {
            _dedMainText[i]    = "";
            _dedFormatText[i]  = "";
            // Sentinel values guarantee a full repaint on the very first render
            _dedRenderedMain[i] = "\xFF";
            _dedRenderedFmt[i]  = "\xFF";
        }

        // Parse display-switching keys from the F-16C-specific options.
        // These are separate from the FA18C NextPageKey/PrevPageKey so the two
        // aircraft can be configured independently.
        _dedDisplayKey = Enum.TryParse<Key>(options.F16CPrevDisplayKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;
        _navDisplayKey = Enum.TryParse<Key>(options.F16CNextDisplayKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _rwrDisplayKey = Enum.TryParse<Key>(options.F16CRwrDisplayKey, out var rwrKey)
            ? rwrKey
            : Key.LineSelectRight1;

        if (mcdu != null)
        {
            mcdu.KeyDown += HandleKeyDown;
        }
    }

    ~F16C_Listener() => Dispose(false);

    // =========================================================================
    // Display switching
    // =========================================================================

    // Keyboard hook — wired in constructor, unwired in Dispose.
    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == _dedDisplayKey)
            SwitchDisplay(DisplayMode.DED);
        else if (e.Key == _navDisplayKey)
            SwitchDisplay(DisplayMode.NAV);
        else if (e.Key == _rwrDisplayKey)
            SwitchDisplay(DisplayMode.RWR);
    }

    // Switches to the requested mode, invalidating the DED dirty-check cache
    // so that the first frame after switching back to DED is always a full repaint.
    public void SwitchDisplay(DisplayMode newMode)
    {
        if (newMode == DisplayMode.DED)
            InvalidateDedCache();

        _currentDisplay = newMode;
        RefreshActiveDisplay();
    }

    // Resets the dirty-check shadows so every DED row is repainted on the next
    // UpdateDEDDisplay() call.  Call whenever the screen may have been overwritten
    // by another display mode.
    private void InvalidateDedCache()
    {
        for (int i = 0; i < 5; i++)
        {
            _dedRenderedMain[i] = "\xFF";
            _dedRenderedFmt[i]  = "\xFF";
        }
    }

    // Unsubscribes the keyboard hook before the base class disposes the MCDU.
    protected override void Dispose(bool disposing)
    {
        if (disposing && mcdu != null)
        {
            mcdu.KeyDown -= HandleKeyDown;
        }
        base.Dispose(disposing);
    }

    // =========================================================================
    // Initialization
    // =========================================================================
    protected override void InitializeDcsBiosControls()
    {
        // --- Console / Shared ---
        _PRI_CONSOLES_BRT     = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_CONSOLES_BRT_KNB"); // 0x440E
        _LIGHT_MASTER_CAUTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_MASTER_CAUTION"); // 0x447A

        // --- DED brightness ---
        _PRI_DATA_DISPLAY_BRT = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("PRI_DATA_DISPLAY_BRT_KNB"); // 0x4412

        // --- DED main lines (0 = LINE_1 … 4 = LINE_5) ---
        _dedMainLines[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_1"); // 0x450A
        _dedMainLines[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_2"); // 0x4528
        _dedMainLines[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_3"); // 0x4546
        _dedMainLines[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_4"); // 0x4564
        _dedMainLines[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_5"); // 0x4582

        // --- DED secondary lines (inverse-only echo) ---
        _dedSecLines[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L1"); // 0x45EE
        _dedSecLines[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L2"); // 0x4606
        _dedSecLines[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L3"); // 0x461E
        _dedSecLines[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L4"); // 0x4636
        _dedSecLines[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L5"); // 0x464E

        // --- DED format masks ('i' = inverse, ' ' = normal) ---
        _dedFormats[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L1_FORMAT"); // 0x4666
        _dedFormats[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L2_FORMAT"); // 0x467E
        _dedFormats[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L3_FORMAT"); // 0x4696
        _dedFormats[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L4_FORMAT"); // 0x46AE
        _dedFormats[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L5_FORMAT"); // 0x46C6

        // --- MFD switches (no framebuffer) ---
        _MFD_L_BRT_SW = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MFD_L_BRT_SW"); // 0x4462
        _MFD_R_BRT_SW = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MFD_R_BRT_SW"); // 0x4464

        // --- EHSI text ---
        _EHSI_COURSE     = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_COURSE");     // 0x45E6
        _EHSI_RANGE      = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_RANGE");      // 0x45EA
        _EHSI_MODE_LEFT  = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_MODE_LEFT");  // 0x45DE
        _EHSI_MODE_RIGHT = DCSBIOSControlLocator.GetStringDCSBIOSOutput("EHSI_MODE_RIGHT"); // 0x45E2

        // --- EHSI integer ---
        _EHSI_RANGE_INVALID      = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_RANGE_INVALID");      // 0x4580
        _EHSI_CRS_SET_KNB        = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_CRS_SET_KNB");       // 0x4470
        _EHSI_HDG_SET_KNB        = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("EHSI_HDG_SET_KNB");       // 0x4472
        _STANDBY_COMPASS_HEADING = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("STANDBY_COMPASS_HEADING");

        _ALT_10000        = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_10000_FT_CNT");
        _ALT_1000         = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_1000_FT_CNT");
        _ALT_100          = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_100_FT_CNT");
        _ALT_PNEU         = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PNEU_FLAG");
        _QNH_D0           = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_0_CNT");
        _QNH_D1           = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_1_CNT");
        _QNH_D2           = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_2_CNT");
        _QNH_D3           = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ALT_PRESSURE_DRUM_3_CNT");
        _AIRSPEED         = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("AIRSPEED");
        _MACH             = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("MACH_INDICATOR");
        _VVI              = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("VVI");
        _FUEL_TOT_10K     = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_10K");
        _FUEL_TOT_1K      = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_1K");
        _FUEL_TOT_100     = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELTOTALIZER_100");
        _FUEL_FF_10K      = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_10K");
        _FUEL_FF_1K       = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_1K");
        _FUEL_FF_100      = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUELFLOWCOUNTER_100");
        _FUEL_AL          = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUEL_AL");
        _FUEL_FR          = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("FUEL_FR");
        _LIGHT_FUEL_LOW   = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_AFT_FUEL_LOW");
        _CLOCK_H          = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_CURRTIME_H");
        _CLOCK_MS         = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_CURRTIME_MS");
        _CLOCK_ELAPSED_M  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_ELAPSED_TIME_M");
        _CLOCK_ELAPSED_S  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("CLOCK_ELAPSED_TIME_SEC");

        _LIGHT_RWR_POWER      = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_POWER");
        _LIGHT_RWR_ACTIVITY   = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ACTIVITY");
        _LIGHT_RWR_SEARCH     = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SEARCH");
        _LIGHT_RWR_ALT        = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ALT");
        _LIGHT_RWR_ALT_LOW    = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_ALT_LOW");
        _LIGHT_RWR_MSL_LAUNCH = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MSL_LAUNCH");
        _LIGHT_RWR_MODE_PRI   = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MODE_PRI");
        _LIGHT_RWR_MODE_OPEN  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_MODE_OPEN");
        _LIGHT_RWR_HANDOFF_H  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_HANDOFF_H");
        _LIGHT_RWR_SHIP_UNK   = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SHIP_UNK");
        _LIGHT_RWR_TGTSEP_DN  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_TGTSEP_DN");
        _LIGHT_RWR_TGTSEP_UP  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_TGTSEP_UP");
        _LIGHT_RWR_SYSTEST    = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_RWR_SYSTEST");
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
        _LIGHT_CMDS_RDY  = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_RDY");
        _LIGHT_CMDS_GO   = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_GO");
        _LIGHT_CMDS_DISP = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("LIGHT_CMDS_DISP");
    }

    // =========================================================================
    // Integer / UInt data handler
    // =========================================================================
    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            bool refreshCdu     = false;
            bool refreshDisplay = false;
            UpdateCounter(e.Address, e.Data);

            // --- MCDU backlight — active regardless of display mode ---
            if (mcdu != null && !options.DisableLightingManagement)
            {
                if (e.Address == _PRI_CONSOLES_BRT!.Address)
                {
                    mcdu.BacklightBrightnessPercent =
                        (int)(_PRI_CONSOLES_BRT!.GetUIntValue(e.Data) * 100 / _PRI_CONSOLES_BRT.MaxValue);
                    refreshCdu = true;
                }

                // Mirror the cockpit DED brightness knob to the MCDU display brightness
                if (e.Address == _PRI_DATA_DISPLAY_BRT!.Address)
                {
                    mcdu.DisplayBrightnessPercent =
                        (int)(_PRI_DATA_DISPLAY_BRT!.GetUIntValue(e.Data) * 100 / _PRI_DATA_DISPLAY_BRT.MaxValue);
                    refreshCdu = true;
                }
            }

            if (frontpanelHub.HasFrontpanels && !options.DisableLightingManagement)
            {
                if (e.Address == _PRI_CONSOLES_BRT!.Address)
                {
                    var b = (byte)(_PRI_CONSOLES_BRT!.GetUIntValue(e.Data) * 255 / _PRI_CONSOLES_BRT.MaxValue);
                    frontpanelHub.SetBrightness(b, b, b);
                }
            }

            // --- Master Caution LED — active regardless of display mode ---
            if (mcdu != null && e.Address == _LIGHT_MASTER_CAUTION!.Address)
            {
                mcdu.Leds.Fail = _LIGHT_MASTER_CAUTION.GetUIntValue(e.Data) == 1;
                refreshCdu = true;
            }

            // --- Display-specific integer data ---
            if (_currentDisplay == DisplayMode.NAV)
            {
                if (e.Address == _EHSI_RANGE_INVALID!.Address)
                {
                    _ehsiRangeInvalid = _EHSI_RANGE_INVALID.GetUIntValue(e.Data) == 1;
                    refreshDisplay    = true;
                }
                if (e.Address == _EHSI_CRS_SET_KNB!.Address)
                {
                    _ehsiCrsDeg    = KnobToDegrees(_EHSI_CRS_SET_KNB.GetUIntValue(e.Data));
                    refreshDisplay = true;
                }
                if (e.Address == _EHSI_HDG_SET_KNB!.Address)
                {
                    _ehsiHdgDeg    = KnobToDegrees(_EHSI_HDG_SET_KNB.GetUIntValue(e.Data));
                    refreshDisplay = true;
                }

                // Altitude drums — each drum updates its own cached digit, then recompose
                if (e.Address == _ALT_10000!.Address)
                {
                    _altD10k = (int)Math.Round(_ALT_10000.GetUIntValue(e.Data) * 9.0 / 65535.0);
                    _altFt = _altD10k * 10000 + _altD1k * 1000 + _altD100 * 100;
                    refreshDisplay = true;
                }
                if (e.Address == _ALT_1000!.Address)
                {
                    _altD1k = (int)Math.Round(_ALT_1000.GetUIntValue(e.Data) * 9.0 / 65535.0);
                    _altFt = _altD10k * 10000 + _altD1k * 1000 + _altD100 * 100;
                    refreshDisplay = true;
                }
                if (e.Address == _ALT_100!.Address)
                {
                    _altD100 = (int)Math.Round(_ALT_100.GetUIntValue(e.Data) * 9.0 / 65535.0);
                    _altFt = _altD10k * 10000 + _altD1k * 1000 + _altD100 * 100;
                    refreshDisplay = true;
                }

                if (e.Address == _ALT_PNEU!.Address)
                { _pneuFail = _ALT_PNEU.GetUIntValue(e.Data) > 32767; refreshDisplay = true; }

                if (e.Address == _QNH_D0!.Address) { _qnhD0 = (int)Math.Round(_QNH_D0.GetUIntValue(e.Data) * 9.0 / 65535.0); refreshDisplay = true; }
                if (e.Address == _QNH_D1!.Address) { _qnhD1 = (int)Math.Round(_QNH_D1.GetUIntValue(e.Data) * 9.0 / 65535.0); refreshDisplay = true; }
                if (e.Address == _QNH_D2!.Address) { _qnhD2 = (int)Math.Round(_QNH_D2.GetUIntValue(e.Data) * 9.0 / 65535.0); refreshDisplay = true; }
                if (e.Address == _QNH_D3!.Address) { _qnhD3 = (int)Math.Round(_QNH_D3.GetUIntValue(e.Data) * 9.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _AIRSPEED!.Address)
                { _iasKts = (int)Math.Round(_AIRSPEED.GetUIntValue(e.Data) * 1000.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _MACH!.Address)
                { _mach = _MACH.GetUIntValue(e.Data) * 0.85 / 65535.0; refreshDisplay = true; }

                if (e.Address == _VVI!.Address)
                { _vviFpm = (int)Math.Round((_VVI.GetUIntValue(e.Data) - 32768.0) * 6000.0 / 32767.0); refreshDisplay = true; }

                if (e.Address == _FUEL_TOT_10K!.Address || e.Address == _FUEL_TOT_1K!.Address || e.Address == _FUEL_TOT_100!.Address)
                {
                    _fuelTotalLb = (int)Math.Round(_FUEL_TOT_10K.GetUIntValue(e.Data) * 9.0 / 65535.0) * 1000
                                 + (int)Math.Round(_FUEL_TOT_1K.GetUIntValue(e.Data)  * 9.0 / 65535.0) * 100
                                 + (int)Math.Round(_FUEL_TOT_100.GetUIntValue(e.Data) * 9.0 / 65535.0) * 10;
                    refreshDisplay = true;
                }

                if (e.Address == _FUEL_FF_10K!.Address || e.Address == _FUEL_FF_1K!.Address || e.Address == _FUEL_FF_100!.Address)
                {
                    _fuelFlowPph = (int)Math.Round(_FUEL_FF_10K.GetUIntValue(e.Data) * 9.0 / 65535.0) * 1000
                                 + (int)Math.Round(_FUEL_FF_1K.GetUIntValue(e.Data)  * 9.0 / 65535.0) * 100
                                 + (int)Math.Round(_FUEL_FF_100.GetUIntValue(e.Data) * 9.0 / 65535.0) * 10;
                    refreshDisplay = true;
                }

                if (e.Address == _FUEL_AL!.Address)
                { _fuelAlLb = (int)Math.Round(_FUEL_AL.GetUIntValue(e.Data) * 4000.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _FUEL_FR!.Address)
                { _fuelFrLb = (int)Math.Round(_FUEL_FR.GetUIntValue(e.Data) * 4000.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _LIGHT_FUEL_LOW!.Address)
                { _fuelLow = _LIGHT_FUEL_LOW.GetUIntValue(e.Data) == 1; refreshDisplay = true; }

                if (e.Address == _CLOCK_H!.Address)
                { _clockH = (int)Math.Round(_CLOCK_H.GetUIntValue(e.Data) * 11.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _CLOCK_MS!.Address)
                { _clockMin = (int)Math.Round(_CLOCK_MS.GetUIntValue(e.Data) * 59.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _CLOCK_ELAPSED_M!.Address)
                { _elapsedMin = (int)Math.Round(_CLOCK_ELAPSED_M.GetUIntValue(e.Data) * 59.0 / 65535.0); refreshDisplay = true; }

                if (e.Address == _CLOCK_ELAPSED_S!.Address)
                { _elapsedSec = (int)Math.Round(_CLOCK_ELAPSED_S.GetUIntValue(e.Data) * 59.0 / 65535.0); refreshDisplay = true; }
            }

            if (e.Address == _STANDBY_COMPASS_HEADING!.Address)
            {
                uint raw = _STANDBY_COMPASS_HEADING.GetUIntValue(e.Data);
                _currentHeadingDeg = (int)Math.Round(raw * 360.0 / 65535.0) % 360;
                if (_currentDisplay == DisplayMode.NAV) UpdateNAVDisplay();
            }

            if (e.Address == _EHSI_RANGE_INVALID!.Address)
            {
                _rangeInvalid = _EHSI_RANGE_INVALID.GetUIntValue(e.Data) == 1;
            }

            if (e.Address == _EHSI_HDG_SET_KNB!.Address)
            {
                uint newVal = _EHSI_HDG_SET_KNB.GetUIntValue(e.Data);
                int delta = (int)newVal - (int)_lastHdgKnobValue;
                if (Math.Abs(delta) < 32768) // ignore wraparound jumps
                    _hdgBugDeg = (_hdgBugDeg + Math.Sign(delta) + 360) % 360;
                _lastHdgKnobValue = newVal;
            }

            // ── RWR / ECM / CMDS lights (always update, not display-gated) ───────
            void L(DCSBIOSOutput? ctrl, ref bool field)
            {
                if (ctrl != null && ctrl.Address == e.Address)
                { field = ctrl.GetUIntValue(e.Data) == 1; refreshDisplay = true; }
            }
            L(_LIGHT_RWR_POWER,      ref _rwrPower);
            L(_LIGHT_RWR_ACTIVITY,   ref _rwrActivity);
            L(_LIGHT_RWR_SEARCH,     ref _rwrSearch);
            L(_LIGHT_RWR_ALT,        ref _rwrAlt);
            L(_LIGHT_RWR_ALT_LOW,    ref _rwrAltLow);
            L(_LIGHT_RWR_MSL_LAUNCH, ref _rwrMslLaunch);
            L(_LIGHT_RWR_MODE_PRI,   ref _rwrModePri);
            L(_LIGHT_RWR_MODE_OPEN,  ref _rwrModeOpen);
            L(_LIGHT_RWR_HANDOFF_H,  ref _rwrHandoff);
            L(_LIGHT_RWR_SHIP_UNK,   ref _rwrShipUnk);
            L(_LIGHT_RWR_TGTSEP_DN,  ref _rwrTgtSepDn);
            L(_LIGHT_RWR_TGTSEP_UP,  ref _rwrTgtSepUp);
            L(_LIGHT_RWR_SYSTEST,    ref _rwrSysTest);
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
            L(_LIGHT_CMDS_RDY,  ref _cmdsRdy);
            L(_LIGHT_CMDS_GO,   ref _cmdsGo);
            L(_LIGHT_CMDS_DISP, ref _cmdsDisp);

            if (refreshDisplay) RefreshActiveDisplay();

            if (refreshCdu && mcdu != null)
            {
                if (!options.DisableLightingManagement) mcdu.RefreshBrightnesses();
                mcdu.RefreshLeds();
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "F-16C: Failed to process DCS-BIOS data");
        }
    }

    // =========================================================================
    // String data handler
    // =========================================================================
    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        try
        {
            switch (_currentDisplay)
            {
                case DisplayMode.DED:
                    // Main lines carry the displayable text
                    if (TryMatchArray(_dedMainLines, e.Address, out int mainIdx))
                    {
                        _dedMainText[mainIdx] = e.StringData;
                        UpdateDEDDisplay();
                        break;
                    }
                    // Format masks drive the inverse-video colouring
                    if (TryMatchArray(_dedFormats, e.Address, out int fmtIdx))
                    {
                        _dedFormatText[fmtIdx] = e.StringData;
                        UpdateDEDDisplay();
                        break;
                    }
                    // Secondary lines (DED_L*) echo only the inverse chars.
                    // We already have the full text from DED_LINE_* and the
                    // positions from the FORMAT mask, so secondary lines are
                    // not needed for rendering — silently ignore them here.
                    break;

                case DisplayMode.NAV:
                    bool hsiChanged = false;
                    if (e.Address == _EHSI_COURSE!.Address)     { _ehsiCourse     = e.StringData.Trim(); hsiChanged = true; }
                    if (e.Address == _EHSI_COURSE!.Address)
                    {
                        if (int.TryParse(e.StringData.Trim(), out int crs))
                            _selectedCourseDeg = crs;
                    }
                    if (e.Address == _EHSI_RANGE!.Address)      { _ehsiRange      = e.StringData.Trim(); hsiChanged = true; }
                    if (e.Address == _EHSI_MODE_LEFT!.Address)  { _ehsiModeLeft   = e.StringData.Trim(); hsiChanged = true; }
                    if (e.Address == _EHSI_MODE_RIGHT!.Address) { _ehsiModeRight  = e.StringData.Trim(); hsiChanged = true; }
                    if (hsiChanged) UpdateNAVDisplay();
                    break;

                case DisplayMode.RWR:
                    bool rwrChanged = false;
                    if (e.Address == _CMDS_CH_AMOUNT!.Address) { _cmdsCh = e.StringData.Trim(); rwrChanged = true; }
                    if (e.Address == _CMDS_FL_AMOUNT!.Address) { _cmdsFl = e.StringData.Trim(); rwrChanged = true; }
                    if (e.Address == _CMDS_O1_AMOUNT!.Address) { _cmdsO1 = e.StringData.Trim(); rwrChanged = true; }
                    if (e.Address == _CMDS_O2_AMOUNT!.Address) { _cmdsO2 = e.StringData.Trim(); rwrChanged = true; }
                    if (rwrChanged) UpdateRWRDisplay();
                    break;
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "F-16C: Failed to process DCS-BIOS string data");
        }
    }

    // =========================================================================
    // DED display — full implementation
    // =========================================================================
    //
    // MCDU layout (14 rows × 24 cols):
    //
    //  Row 0  │ DED_LINE_1   green text; amber-on-BGGreen = inverse video
    //  Row 1  │ DED_LINE_2
    //  Row 2  │ DED_LINE_3
    //  Row 3  │ DED_LINE_4
    //  Row 4  │ DED_LINE_5
    //  Rows 5–13 → cleared
    //
    // Inverse-video simulation:
    //   The real DED uses bright-on-dark green phosphor for highlighted fields.
    //   On the MCDU LCD we render highlighted chars as  Amber text on Green
    //   background — visually distinct from the Green-on-Black baseline.
    //   The Compositor supports per-cell background colour, so each run of
    //   consecutive same-format chars is written as one Write() call.
    //
    // Dirty check:
    //   Only rows where (text, format) differ from the last painted values are
    //   rewritten.  Most BIOS update cycles touch ≤ 2 rows, so the LCD bus
    //   stays idle on stable DED pages.
    // =========================================================================
    private void UpdateDEDDisplay()
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);

        for (int i = 0; i < 5; i++)
        {
            string text = NormaliseDedLine(ApplyDedCharacterMap(_dedMainText[i]));
            string fmt  = _dedFormatText[i];

            // Skip if unchanged since last draw
            if (text == _dedRenderedMain[i] && fmt == _dedRenderedFmt[i])
                continue;

            _dedRenderedMain[i] = text;
            _dedRenderedFmt[i]  = fmt;

            RenderDedLine(output, i, text, fmt);
        }

        // Clear rows 5–12 to erase any content left by other display modes
        for (int r = 5; r < Metrics.Lines - 1; r++)
        {
            output.Line(r).BGBlack().Green().Write(new string(' ', Metrics.Columns));
        }

        // Row 13 — mode indicator
        output.Line(Metrics.Lines - 1).BGBlack().Green()
              .Write(new string(' ', Metrics.Columns - 5))
              .Amber().Write("[DED]");
    }

    // ─── RenderDedLine ────────────────────────────────────────────────────────
    //
    // Renders one DED row onto the compositor by splitting it into runs of
    // consecutive same-format characters and writing each run with the
    // appropriate fore/background colour pair.
    //
    // Colour scheme:
    //   Normal   → Green text   on Black background  (standard DED phosphor)
    //   Inverse  → Black text   on Green background  (simulates DED bright-field)
    //
    // The Compositor.Write() method advances the column cursor without moving to
    // the next row, enabling multiple colour segments on the same row.
    // ─────────────────────────────────────────────────────────────────────────
    private static void RenderDedLine(Compositor output, int row, string text, string fmt)
    {
        // Pad the format mask to the text width; any missing positions are normal.
        string safeFmt = fmt.PadRight(text.Length);

        output.Line(row); // position cursor at col 0, row N

        int pos = 0;
        while (pos < text.Length)
        {
            bool isInverse = safeFmt[pos] == 'i';
            int  start     = pos;

            // Extend the run while the format character matches.
            while (pos < text.Length && (safeFmt[pos] == 'i') == isInverse)
                pos++;

            string segment = text[start..pos];

            if (isInverse)
                output.Black().BGGreen().Write(segment);
            else
                output.Green().BGBlack().Write(segment);
        }
    }

    // =========================================================================
    // MFD display stubs — no framebuffer available from DCS-BIOS
    // =========================================================================
    private void UpdateMFD1Display()
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);
        output.Clear().Line(0).Amber().Write("[LEFT MFD]")
              .Line(2).Green().Write("No framebuffer data.")
              .Line(3).Green().Write("Add derived BIOS outputs")
              .Line(4).Green().Write("to populate this screen.");
    }

    private void UpdateMFD2Display()
    {
        if (mcdu == null) return;
        var output = GetCompositor(DEFAULT_PAGE);
        output.Clear().Line(0).Amber().Write("[RIGHT MFD]")
              .Line(2).Green().Write("No framebuffer data.")
              .Line(3).Green().Write("Add derived BIOS outputs")
              .Line(4).Green().Write("to populate this screen.");
    }

    // =========================================================================
    // HSI / EHSI display
    // =========================================================================
    //
    // MCDU layout  (14 rows × 24 cols):
    //
    //  Row  0  "      F-16C EHSI      "  header, amber, centered
    //  Row  1  "PLS              NAV  "  mode annunciators, left/right
    //  Row  2  "========================"  separator
    //  Row  3  "CRS 045   HDG 270     "  course text + heading bug
    //  Row  4  "RNG     12.5 nm       "  range (red "----" when invalid)
    //  Row  5  "------------------------"  separator
    //  Row  6  "           N            "  ┐
    //  Row  7  "       NW      NE       "  │  compass rose
    //  Row  8  "    W      +      E     "  │  active sector = Amber on BGGreen
    //  Row  9  "       SW      SE       "  │  rest = Green on BGBlack
    //  Row 10  "           S            "  ┘
    //  Row 11  "BRG: NE  (045)          "  8-direction label + numeric bearing
    //  Row 12  (blank)
    //  Row 13  (blank)
    //
    // Compass bearing shown is the COURSE SET knob position (_ehsiCrsDeg),
    // not the aircraft heading (which DCS-BIOS does not export for the F-16C EHSI).
    // =========================================================================
    private void UpdateNAVDisplay()
    {
        try
        {
            var o = GetCompositor(DEFAULT_PAGE);

            // Row 0 — header
            o.Line(0).Amber().WriteLine($"F-16C NAV   HDG:{_currentHeadingDeg:D3}");

            // Row 1 — navigation: range and course
            string rng  = _ehsiRangeInvalid ? "---" : (_ehsiRange.Length > 0 ? _ehsiRange : "---");
            string crs  = _ehsiCourse.Length > 0 ? _ehsiCourse : "---";
            o.Line(1).Green().WriteLine($"RNG:{rng,6}nm    CRS:{crs,3}");

            // Row 2 — nav mode
            string modeL = _ehsiModeLeft.Length  > 0 ? _ehsiModeLeft  : "   ";
            string modeR = _ehsiModeRight.Length > 0 ? _ehsiModeRight : "NAV";
            o.Line(2).Green().WriteLine($"MODE:{modeL} {modeR,-18}");

            // Row 3 — separator
            o.Line(3).Green().WriteLine(new string('-', 24));

            // Row 4 — altitude and vertical speed
            string vs = _vviFpm >= 0 ? $"+{_vviFpm,5}" : $"{_vviFpm,6}";
            o.Line(4).Green().WriteLine($"ALT:{_altFt,6}ft VS:{vs}");

            // Row 5 — airspeed and mach
            o.Line(5).Green().WriteLine($"IAS:{_iasKts,6}kt MCH:{_mach:F2}");

            // Row 6 — QNH and pneumatic flag
            string qnh  = $"{_qnhD3}{_qnhD2}.{_qnhD1}{_qnhD0}";
            string pneu = _pneuFail ? "FAIL" : " OK ";
            o.Line(6).Green().WriteLine($"QNH:{qnh,6}in PNU:{pneu}");

            // Row 7 — separator
            o.Line(7).Green().WriteLine(new string('-', 24));

            // Row 8 — fuel total and flow
            o.Line(8).Green().WriteLine($"TOT:{_fuelTotalLb,5}lb FF:{_fuelFlowPph,5}");

            // Row 9 — tank breakdown
            o.Line(9).Green().WriteLine($"AFT:{_fuelAlLb,5}lb FWD:{_fuelFrLb,4}");

            // Row 10 — endurance calculation
            double endurH = _fuelFlowPph > 0 ? (double)_fuelTotalLb / _fuelFlowPph : 0;
            int endurMin  = (int)(endurH * 60);
            int rangeNm   = (int)(endurH * 300); // 300kt groundspeed estimate
            o.Line(10).Green().WriteLine($"END:{endurMin/60}h{endurMin%60:D2}  RNG~{rangeNm,4}nm");

            // Row 11 — separator
            o.Line(11).Green().WriteLine(new string('-', 24));

            // Row 12 — clock and hack time
            int utcHour = DateTime.UtcNow.Hour;
            o.Line(12).Green().WriteLine($"TIME:{utcHour:D2}:{_clockMin:D2}Z  HACK:{_elapsedMin:D2}:{_elapsedSec:D2}");

            // Row 13 — fuel warning (amber if active, blank if not) + mode tag
            if (_fuelLow)
                o.Line(13).Amber().WriteLine("** FUEL LOW **      [NAV]");
            else
                o.Line(13).Green().WriteLine($"{"[NAV]",24}");
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "NAV display update failed");
        }
    }

    // ─── RenderCompassRose ────────────────────────────────────────────────────
    //
    // Draws a 5-row, 8-direction ASCII compass rose onto the compositor,
    // starting at `startRow`.  The sector nearest to `courseDeg` is highlighted
    // in Amber-on-BGGreen; all other directions are Green-on-BGBlack.
    //
    // Compass layout (each string is exactly Metrics.Columns = 24 chars):
    //
    //   "           N            "   N  at col 11
    //   "       NW      NE       "   NW at cols 7–8,  NE at cols 15–16
    //   "    W      +      E     "   W  at col 4,  +  at col 11,  E at col 18
    //   "       SW      SE       "   SW at cols 7–8,  SE at cols 15–16
    //   "           S            "   S  at col 11
    //
    // Bearing is the CRS knob position, not aircraft heading (not exported by BIOS).
    // ─────────────────────────────────────────────────────────────────────────
    private static readonly string[] CompassBaseRows =
    {
        "           N            ",   // rose row 0
        "       NW      NE       ",   // rose row 1
        "    W      +      E     ",   // rose row 2
        "       SW      SE       ",   // rose row 3
        "           S            ",   // rose row 4
    };

    // (roseRowIndex, colStart, length) for each of the 8 compass sectors.
    // Sector index = ((degrees + 22) / 45) % 8  →  0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
    private static readonly (int Row, int Col, int Len)[] CompassSectorPos =
    {
        (0, 11, 1),  // N
        (1, 15, 2),  // NE
        (2, 18, 1),  // E
        (3, 15, 2),  // SE
        (4, 11, 1),  // S
        (3,  7, 2),  // SW
        (2,  4, 1),  // W
        (1,  7, 2),  // NW
    };

    private static void RenderCompassRose(Compositor c, int courseDeg, int startRow)
    {
        int sector = ((courseDeg + 22) / 45) % 8;
        var (activeRoseRow, activeCol, activeLen) = CompassSectorPos[sector];

        for (int ri = 0; ri < CompassBaseRows.Length; ri++)
        {
            string rowText = CompassBaseRows[ri];
            c.Line(startRow + ri);

            if (ri != activeRoseRow)
            {
                // Non-active rows: write entirely in Green
                c.Green().BGBlack().Write(rowText);
            }
            else
            {
                // Active row: three segments — before / highlight / after
                if (activeCol > 0)
                    c.Green().BGBlack().Write(rowText[..activeCol]);

                c.Amber().BGGreen().Write(rowText[activeCol..(activeCol + activeLen)]);

                int afterStart = activeCol + activeLen;
                if (afterStart < rowText.Length)
                    c.Green().BGBlack().Write(rowText[afterStart..]);
            }
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private void RefreshActiveDisplay()
    {
        switch (_currentDisplay)
        {
            case DisplayMode.DED: UpdateDEDDisplay(); break;
            case DisplayMode.NAV: UpdateNAVDisplay(); break;
            case DisplayMode.RWR: UpdateRWRDisplay(); break;
        }
    }

    private void UpdateRWRDisplay()
    {
        try
        {
            var o = GetCompositor(DEFAULT_PAGE);

            // Helper: returns indicator string
            string Ind(bool on) => on ? "ON " : "-- ";

            // ── Row 0: header ─────────────────────────────────────────
            o.Line(0).Amber().WriteLine("F-16C RWR / EW STATUS   ");

            // ── Row 1: separator ──────────────────────────────────────
            o.Line(1).Green().WriteLine(new string('-', 24));

            // ── Row 2: RWR power and mode ─────────────────────────────
            string mode = _rwrModePri ? "PRI" : _rwrModeOpen ? "OPN" : "---";
            o.Line(2).Green().Write("PWR:");
            if (_rwrPower) o.Amber().Write("ON "); else o.Green().Write("-- ");
            o.Green().Write("  MODE:");
            if (_rwrModePri || _rwrModeOpen) o.Amber().Write(mode); else o.Green().Write(mode);
            o.Green().WriteLine("  ");

            // ── Row 3: activity, search ───────────────────────────────
            o.Line(3).Green().Write("ACT:");
            if (_rwrActivity) o.Amber().Write("YES"); else o.Green().Write("-- ");
            o.Green().Write("  SRCH:");
            if (_rwrSearch) o.Amber().Write("YES"); else o.Green().Write("-- ");
            o.Green().WriteLine("    ");

            // ── Row 4: separator ──────────────────────────────────────
            o.Line(4).Green().WriteLine(new string('-', 24));

            // ── Row 5: MSL LAUNCH warning (amber when active) ─────────
            if (_rwrMslLaunch)
                o.Line(5).Black().BGAmber().WriteLine("  *** MSL LAUNCH ***    ");
            else
                o.Line(5).Green().WriteLine("  [ no launch detected ]");

            // ── Row 6: ALT box and ALT low ────────────────────────────
            o.Line(6).Green().Write("ALT:");
            if (_rwrAlt)    o.Amber().Write("ACT "); else o.Green().Write("--- ");
            o.Green().Write("ALTLOW:");
            if (_rwrAltLow) o.Amber().Write("YES"); else o.Green().Write("-- ");
            o.Green().WriteLine(" ");

            // ── Row 7: handoff, ship unknown, tgt sep ─────────────────
            o.Line(7).Green()
             .Write($"HND:{Ind(_rwrHandoff)} ")
             .Write($"UNK:{Ind(_rwrShipUnk)} ")
             .Write($"SEP:{(_rwrTgtSepDn ? "DN" : _rwrTgtSepUp ? "UP" : "--")}");

            // ── Row 8: separator ──────────────────────────────────────
            o.Line(8).Green().WriteLine(new string('-', 24));

            // ── Row 9: ECM pod header ─────────────────────────────────
            o.Line(9).Green().WriteLine("ECM  A  F  S  T         ");

            // ── Rows 10-14: ECM pods 1-5 ─────────────────────────────
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

            // ── Last row: CMDS countermeasures + mode tag ─────────────
            o.Line(Metrics.Lines - 1).Green()
             .Write($"CH:{_cmdsCh,3} FL:{_cmdsFl,3} ")
             .Amber().Write("[RWR]");
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "RWR display update failed");
        }
    }

    // Returns true + the array index when `address` matches one output in the array.
    private static bool TryMatchArray(DCSBIOSOutput?[] outputs, uint address, out int index)
    {
        for (int i = 0; i < outputs.Length; i++)
        {
            if (outputs[i] != null && outputs[i]!.Address == address)
            {
                index = i;
                return true;
            }
        }
        index = -1;
        return false;
    }

    // Maps a 0–65535 BIOS knob position linearly to 0–359 degrees.
    private static int KnobToDegrees(uint raw) =>
        (int)Math.Round(raw / 65535.0 * 359.0) % 360;

    // Returns the nearest 8-point compass label for a 0–359° bearing.
    // Breakpoints: N=337–22, NE=23–67, E=68–112, SE=113–157,
    //              S=158–202, SW=203–247, W=248–292, NW=293–336
    private static string DegreesToCardinal(int deg)
    {
        string[] labels = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
        return labels[((deg + 22) / 45) % 8];
    }

    // Applies the DED character substitution table to a raw BIOS string.
    private static string ApplyDedCharacterMap(string raw)
    {
        if (string.IsNullOrEmpty(raw)) return string.Empty;
        var sb = new System.Text.StringBuilder(raw.Length);
        foreach (char c in raw)
            sb.Append(DedCharMap.TryGetValue(c, out char mapped) ? mapped : c);
        return sb.ToString();
    }

    // Ensures every row written to the LCD is exactly Metrics.Columns (24) wide.
    // Prevents ghost characters when shorter strings follow longer ones.
    private static string NormaliseDedLine(string text) =>
        text.Length >= Metrics.Columns
            ? text[..Metrics.Columns]
            : text.PadRight(Metrics.Columns);

    private (int x, int y) CompassPoint(int directionDeg, int headingDeg, int cx, int cy, int radius)
    {
        double relAngle = (directionDeg - headingDeg + 360) % 360;
        double rad = relAngle * Math.PI / 180.0;
        int x = cx + (int)Math.Round(radius * Math.Sin(rad));
        int y = cy - (int)Math.Round(radius * 0.5 * Math.Cos(rad)); // 0.5 corrects char aspect ratio
        return (x, y);
    }

    private float CalcDeviation(int headingDeg, int courseDeg)
    {
        float dev = ((headingDeg - courseDeg + 180 + 360) % 360) - 180f;
        return Math.Clamp(dev, -10f, 10f);
    }

    private bool CalcToFlag(int headingDeg, int courseDeg)
    {
        float diff = ((headingDeg - courseDeg + 180 + 360) % 360) - 180f;
        return Math.Abs(diff) < 90f;
    }

    private char GetNeedleChar(double angleRelativeDeg)
    {
        // Normalize to 0-360
        double a = ((angleRelativeDeg % 360) + 360) % 360;

        if (a < 22.5 || a >= 337.5) return '│'; // straight up/down
        if (a < 67.5)  return '/';              // upper-right
        if (a < 112.5) return '─';              // right
        if (a < 157.5) return '\\';             // lower-right
        if (a < 202.5) return '│';              // straight down
        if (a < 247.5) return '/';              // lower-left
        if (a < 292.5) return '─';             // left
        return '\\';                            // upper-left
    }
}
