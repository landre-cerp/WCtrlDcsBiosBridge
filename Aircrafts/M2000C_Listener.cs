using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using System;
using System.Collections.Generic;
using System.Linq;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal class M2000C_Listener : AircraftListener
{
    // --- Register addresses ---
    // The PCN/CLP light registers and gear registers are read via RegisterRaw(address, handler).
    // Named DCS-BIOS outputs for these registers either do not exist in the M-2000C module JSON or
    // have incorrect mask/shift definitions, so the standard Register() path cannot be used.
    // RegisterRaw passes the unmasked 16-bit register value to the callback; individual bits are
    // extracted using the masks defined below.
    // If the upstream DCS-BIOS definitions are corrected, these RegisterRaw calls can be replaced
    // with Register(DCSBIOSOutput?, handler) using the corresponding named outputs.
    private const uint LANDING_GEAR_LEVER_ADDR = 0x72a4;
    private const uint GEAR_CONF_AUX_ADDR = 0x7244;

    private const uint PCN_LIGHTS_ADDRESS = 29380;
    private const uint PCN_LIGHTS_ADDRESS_2 = 29384;
    private const uint CLP_ADDR_1 = 29248; 
    private const uint CLP_ADDR_2 = 29238;
    private const uint CLP_ADDR_3 = 29250;

    // --- Gear lever light mask ---
    private const uint MASK_GEAR_LEVER_LIGHT  = 0x0400; 
    private const uint MASK_GEAR_CONF_ROUGE = 0x0200;  
    private const uint MASK_GEAR_CONF_G     = 0x0400;  
    private const uint MASK_GEAR_CONF_AUX   = 0x0800;  
    private const uint MASK_GEAR_CONF_D     = 0x1000;  

    // --- PCN lights masks (bit flags in PCN_LIGHTS_ADDRESS register) ---
    private const uint MASK_ALN = 4096;
    private const uint MASK_PRET = 2048;
    private const uint MASK_NDEG = 16384;
    private const uint MASK_MIP = 8192;
    private const uint MASK_SEC = 32768;
    private const uint MASK_UNI = 256;

    // --- CLP masks (Caution Light Panel) - Register 29248 (CLP_ADDR_1) ---
    private const uint MASK_CLP_BP_D = 1;
    private const uint MASK_CLP_TRANSF = 2;
    private const uint MASK_CLP_NIVEAU = 4; 
    private const uint MASK_CLP_HYD_S = 32;
    private const uint MASK_CLP_EP = 64;     
    private const uint MASK_CLP_BINGO = 128;
    private const uint MASK_CLP_P_CAB = 256;
    private const uint MASK_CLP_TEMP = 512;
    private const uint MASK_CLP_REG_O2 = 1024;
    private const uint MASK_CLP_5MN_O2 = 2048;
    // From DCS-BIOS JSON: CLP_O2_HA at address 29248 mask 4096 (shift 12)
    private const uint MASK_CLP_O2_HA = 4096;
    private const uint MASK_CLP_ANEMO = 8192;
    private const uint MASK_CLP_CC = 16384;
    private const uint MASK_CLP_DSV = 32768;

    // --- CLP masks - Register 29238 (CLP_ADDR_2) ---
    private const uint MASK_CLP_HYD_1 = 8;
    private const uint MASK_CLP_HYD_2 = 16;
    private const uint MASK_CLP_BATT = 32;
    private const uint MASK_CLP_TRN = 64; 
    private const uint MASK_CLP_ALT_1 = 128;
    private const uint MASK_CLP_ALT_2 = 256;
    private const uint MASK_CLP_HUILE = 512;
    private const uint MASK_CLP_T7 = 1024;
    private const uint MASK_CLP_CALC = 2048;
    private const uint MASK_CLP_SOURIS = 4096;
    private const uint MASK_CLP_PELLES = 8192;
    private const uint MASK_CLP_BP = 16384;
    private const uint MASK_CLP_BP_G = 32768;

    // --- CLP masks - Register 29250 (CLP_ADDR_3) ---
    private const uint MASK_CLP_CONDIT = 1;
    private const uint MASK_CLP_CONF = 2;
    private const uint MASK_CLP_PA = 4;
    private const uint MASK_CLP_MAN = 8;
    private const uint MASK_CLP_DOM = 16;
    private const uint MASK_CLP_BECS = 32;
    private const uint MASK_CLP_INCIDENCE = 128; 
    private const uint MASK_CLP_GAIN = 256;
    private const uint MASK_CLP_RPM = 512;
    private const uint MASK_CLP_DECOL = 1024;
    private const uint MASK_CLP_PARK = 2048;


    private string _pcnDispL = "N00.00.0";
    private string _pcnDispR = "E00.00.0";
    private string _pcnPrep = "P-1"; 
    private string _pcnDest = "D-1"; 

    private string _clockH= " ";
    private string _clockM= " ";
    private string _clockS= " ";

    private ushort _clpValue1 = 0;
    private ushort _clpValue2 = 0;
    private ushort _clpValue3 = 0;

    private enum CautionSeverity { Advisory, Warning, Critical }
    private readonly struct CautionItem
    {
        public readonly string Text;
        public readonly CautionSeverity Severity;
        public CautionItem(string text, CautionSeverity severity)
        {
            Text = text; Severity = severity;
        }
        public bool IsCritical => Severity == CautionSeverity.Critical;
        public bool IsWarning => Severity == CautionSeverity.Warning;
    }

    // Reusable buffer to avoid allocations every frame
    private readonly List<CautionItem> _cautionBuffer = new(32);

    public M2000C_Listener(UserOptions options) : base(AircraftRegistry.M2000C, options)
    {
    }

    protected override void RegisterCduControls()
    {
        RegisterRaw(PCN_LIGHTS_ADDRESS, data =>
        {
            ushort val = (ushort)data;
            SetCduLeds(
                fm1: (val & MASK_ALN) > 0,
                rdy: (val & MASK_PRET) > 0,
                fm: (val & MASK_NDEG) > 0,
                ind: (val & MASK_MIP) > 0,
                fm2: (val & MASK_SEC) > 0);
        });

        RegisterRaw(PCN_LIGHTS_ADDRESS_2, data =>
        {
            ushort val = (ushort)data;
            SetCduLeds(fail: (val & MASK_UNI) > 0);
        });

        RegisterRaw(CLP_ADDR_1, data =>
        {
            ushort val = (ushort)data;
            if (_clpValue1 != val) { _clpValue1 = val; UpdateCautionPanel(); }
        });

        RegisterRaw(CLP_ADDR_2, data =>
        {
            ushort val = (ushort)data;
            if (_clpValue2 != val) { _clpValue2 = val; UpdateCautionPanel(); }
        });

        RegisterRaw(CLP_ADDR_3, data =>
        {
            ushort val = (ushort)data;
            if (_clpValue3 != val) { _clpValue3 = val; UpdateCautionPanel(); }
        });

        RegisterStr("PCN_DISP_L", s =>
        {
            _pcnDispL = s;
            UpdateCombinedPCNDisplay(GetCompositor(DEFAULT_PAGE));
        });

        RegisterStr("PCN_DISP_R", s =>
        {
            _pcnDispR = s;
            UpdateCombinedPCNDisplay(GetCompositor(DEFAULT_PAGE));
        });

        RegisterStr("PCN_DISP_PREP", s =>
        {
            _pcnPrep = s;
            UpdateCombinedPrepDestDisplay(GetCompositor(DEFAULT_PAGE));
        });

        RegisterStr("PCN_DISP_DEST", s =>
        {
            _pcnDest = s;
            UpdateCombinedPrepDestDisplay(GetCompositor(DEFAULT_PAGE));
        });
    }

    protected override void RegisterFrontpanelControls()
    {
        FlightDeck.SegmentBrightnessPercent = 100; // Default to full brightness until we get a DCS-BIOS update with the brightness control

        RegisterRaw(LANDING_GEAR_LEVER_ADDR, data =>
        {
            FlightDeck.GearWarning = ((ushort)data & MASK_GEAR_LEVER_LIGHT) != 0;
        });

        RegisterRaw(GEAR_CONF_AUX_ADDR, data =>
        {
            ushort val = (ushort)data;
            FlightDeck.GearWarning   = (val & MASK_GEAR_CONF_ROUGE) != 0;
            FlightDeck.GearLeftDown  = (val & MASK_GEAR_CONF_G)     != 0;
            FlightDeck.GearNoseDown  = (val & MASK_GEAR_CONF_AUX)   != 0;
            FlightDeck.GearRightDown = (val & MASK_GEAR_CONF_D)     != 0;
        });

        RegisterUInt("CLK_H", data =>
        {
            _clockH = decodeHourNeedle(data);
            FlightDeck.ClockUtcTime = combineHMS();
        });
        RegisterUInt("CLK_M", data =>
        {
            _clockM = decodeMinuteNeedle(data);
            _clockS = decodeSecondNeedle(data);
            FlightDeck.ClockUtcTime = combineHMS();

        });
        RegisterUInt("CLK_S", data =>
        {
            // CLK_S is the chronograph needle (0–15 min range, not a true seconds hand).
            int totalSeconds = (int)(data / 65535.0 * 900); // 15 min = 900s
            int minutes = totalSeconds / 60;
            int seconds = totalSeconds % 60;
            FlightDeck.ClockChrono = $"{minutes:D2}{seconds:D2}";

        });
    }

    private string decodeSecondNeedle(uint data)
    {
        double fraction = (data * 60 / 65536.0) - (int)(data * 60 / 65536);
        return ((int)(fraction * 60)).ToString("D2");
    }

    private string decodeMinuteNeedle(uint data)
    {
        return ((int)(data * 60 / 65536)).ToString("D2"); 
    }

    private string decodeHourNeedle(uint data)
    {
        return ((int)(data * 12 / 65536)).ToString("D2"); 

    }

    private void UpdateCautionPanel()
    {
        if (!HasCdu) return;
        
        _cautionBuffer.Clear();
        DecodeRegister29238(_clpValue2, _cautionBuffer);
        DecodeRegister29248(_clpValue1, _cautionBuffer);
        DecodeRegister29250(_clpValue3, _cautionBuffer);
        // Priority sorting: Critical > Warning > Advisory, stable within same severity
        _cautionBuffer.Sort(static (a,b) =>
            a.Severity == b.Severity ? 0 : a.Severity switch
            {
                CautionSeverity.Critical => -1, // a first
                CautionSeverity.Warning => b.Severity == CautionSeverity.Critical ? 1 : -1,
                _ => 1 // Advisory last
            });
        RenderCautions(_cautionBuffer);
    }

    private static void DecodeRegister29238(ushort value, List<CautionItem> items)
    {
        // Hydraulic / power / generic cautions
        if ((value & MASK_CLP_HUILE) != 0) items.Add(new("HUILE", CautionSeverity.Critical));
        if ((value & MASK_CLP_BP) != 0) items.Add(new("B.P.", CautionSeverity.Critical));
        if ((value & MASK_CLP_T7) != 0) items.Add(new("T7", CautionSeverity.Critical));
        if ((value & MASK_CLP_BATT) != 0) items.Add(new("BATT", CautionSeverity.Warning));
        if ((value & MASK_CLP_TRN) != 0) items.Add(new("TR", CautionSeverity.Advisory));
        if ((value & MASK_CLP_ALT_1) != 0) items.Add(new("ALT.1", CautionSeverity.Advisory));
        if ((value & MASK_CLP_ALT_2) != 0) items.Add(new("ALT.2", CautionSeverity.Advisory));
        if ((value & MASK_CLP_CALC) != 0) items.Add(new("CALC", CautionSeverity.Advisory));
        if ((value & MASK_CLP_SOURIS) != 0) items.Add(new("SOURIS", CautionSeverity.Advisory));
        if ((value & MASK_CLP_PELLES) != 0) items.Add(new("PELLE", CautionSeverity.Advisory));
        if ((value & MASK_CLP_BP_G) != 0) items.Add(new("BP.G", CautionSeverity.Advisory));
        if ((value & MASK_CLP_HYD_1) != 0) items.Add(new("HYD.1", CautionSeverity.Warning));
        if ((value & MASK_CLP_HYD_2) != 0) items.Add(new("HYD.2", CautionSeverity.Warning));
    }

    private static void DecodeRegister29248(ushort value, List<CautionItem> items)
    {
        // Fuel / oxygen / transfer related cautions
        if ((value & MASK_CLP_DSV) != 0) items.Add(new("DSV", CautionSeverity.Critical));
        if ((value & MASK_CLP_HYD_S) != 0) items.Add(new("HYD.S", CautionSeverity.Critical));
        if ((value & MASK_CLP_P_CAB) != 0) items.Add(new("P.CAB", CautionSeverity.Critical));
        if ((value & MASK_CLP_REG_O2) != 0) items.Add(new("REG.O2", CautionSeverity.Critical));
        if ((value & MASK_CLP_EP) != 0) items.Add(new("EP", CautionSeverity.Critical));
        if ((value & MASK_CLP_O2_HA) != 0) items.Add(new("O2 HA", CautionSeverity.Warning)); // Yellow per JSON description
        if ((value & MASK_CLP_ANEMO) != 0) items.Add(new("ANEMO", CautionSeverity.Advisory));
        if ((value & MASK_CLP_CC) != 0) items.Add(new("CC", CautionSeverity.Advisory));
        if ((value & MASK_CLP_TEMP) != 0) items.Add(new("TEMP", CautionSeverity.Advisory));
        if ((value & MASK_CLP_5MN_O2) != 0) items.Add(new("5mn.O2", CautionSeverity.Warning));
        if ((value & MASK_CLP_TRANSF) != 0) items.Add(new("TRANSF", CautionSeverity.Advisory));
        if ((value & MASK_CLP_BINGO) != 0) items.Add(new("BINGO", CautionSeverity.Warning));
        if ((value & MASK_CLP_NIVEAU) != 0) items.Add(new("NIVEAU", CautionSeverity.Advisory));
        if ((value & MASK_CLP_BP_D) != 0) items.Add(new("BP.D", CautionSeverity.Advisory));
    }

    private static void DecodeRegister29250(ushort value, List<CautionItem> items)
    {
        // Flight control & configuration cautions
        if ((value & MASK_CLP_PA) != 0) items.Add(new("PA", CautionSeverity.Critical));
        if ((value & MASK_CLP_GAIN) != 0) items.Add(new("GAIN", CautionSeverity.Critical));
        if ((value & MASK_CLP_DOM) != 0) items.Add(new("DOM", CautionSeverity.Critical));
        if ((value & MASK_CLP_CONDIT) != 0) items.Add(new("CONDIT", CautionSeverity.Critical));
        if ((value & MASK_CLP_RPM) != 0) items.Add(new("RPM", CautionSeverity.Critical));
        if ((value & MASK_CLP_DECOL) != 0) items.Add(new("DECOL", CautionSeverity.Critical));
        if ((value & MASK_CLP_MAN) != 0) items.Add(new("MAN", CautionSeverity.Advisory));
        if ((value & MASK_CLP_BECS) != 0) items.Add(new("BECS", CautionSeverity.Advisory));
        if ((value & MASK_CLP_CONF) != 0) items.Add(new("CONF", CautionSeverity.Advisory));
        if ((value & MASK_CLP_PARK) != 0) items.Add(new("PARK", CautionSeverity.Advisory));
        if ((value & MASK_CLP_INCIDENCE) != 0) items.Add(new("ALPHA", CautionSeverity.Advisory));
    }

    private void RenderCautions(List<CautionItem> items)
    {
        if (!HasCdu) return;
        
        var output = GetCompositor(DEFAULT_PAGE);
        const int COL_WIDTH_1 = 7;
        const int COL_WIDTH_2 = 7;
        const int COL_WIDTH_3 = 6;
        int currentLine = 0;

        void WriteColumn(Compositor line, CautionItem? item, int colWidth)
        {
            if (item.HasValue)
            {
                string text = item.Value.Text.Length > colWidth ? item.Value.Text.Substring(0, colWidth) : item.Value.Text;
                line = item.Value.Severity switch
                {
                    CautionSeverity.Critical => line.Red(),
                    CautionSeverity.Warning => line.Yellow(),
                    _ => line.Yellow() // Advisory uses yellow; no green on M-2000C CLP
                };
                line.Write(text);
                int padding = colWidth - text.Length;
                if (padding > 0) line.Yellow().Write(new string(' ', padding));
            }
            else
            {
                line.Yellow().Write(new string(' ', colWidth));
            }
        }

        for (int i = 0; i < items.Count; i += 3)
        {
            if (currentLine > 9) break;
            var lineObj = output.Line(currentLine);
            lineObj.ClearRow();

            CautionItem? c1 = i < items.Count ? items[i] : null;
            CautionItem? c2 = (i + 1) < items.Count ? items[i + 1] : null;
            CautionItem? c3 = (i + 2) < items.Count ? items[i + 2] : null;

            WriteColumn(lineObj, c1, COL_WIDTH_1);
            WriteColumn(lineObj, c2, COL_WIDTH_2);
            WriteColumn(lineObj, c3, COL_WIDTH_3);

            currentLine++;
        }

        for (int j = currentLine; j <= 9; j++)
        {
            output.Line(j).ClearRow();
        }

        }

    private void UpdateCombinedPCNDisplay(Compositor output)
    {
        string combinedLine = string.Format("{0}{1,10}", _pcnDispL, _pcnDispR);
        output.Line(12).Green().WriteLine(combinedLine);
    }

    private void UpdateCombinedPrepDestDisplay(Compositor output)
    {
        string prep = ("P:" + _pcnPrep).PadRight(7).Substring(0,7); // e.g. P:XX padded
        string dest = ("D:" + _pcnDest).PadRight(7).Substring(0,7);
        string combinedLine = prep + dest; // 14 chars
        output.Line(13).Green().WriteLine(combinedLine);
    }

    private string combineHMS()
    {
        return $"{_clockH}{_clockM}{_clockS}";
    }
}