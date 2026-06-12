using DCS_BIOS.ControlLocator;
using DCS_BIOS.EventArgs;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class M2000C_Listener : AircraftListener
{
    // --- Register addresses ---
    // Rationale: During early integration a bug was observed where using the official named DCS-BIOS outputs for the M‑2000C
    // lights resulted in no LED updates on the physical MCDU (the callbacks fired, but the decoded light states stayed false).
    // Root cause (still under investigation) appears to be a mismatch between exported module definitions and runtime memory
    // layout for the PCN/CLP light registers. As a pragmatic workaround we bind to existing string display outputs
    // (PCN_DISP_*) purely as container objects and forcibly override their Address with the raw register values.
    // This bypasses the faulty name resolution while reusing the common listener infrastructure. Each CLP register
    // (CLP_ADDR_1..3) and the PCN lights registers (PCN_LIGHTS_ADDRESS / PCN_LIGHTS_ADDRESS_2) is a 16-bit bitfield; individual
    // bits are translated using the masks defined below. If the upstream DCS-BIOS definitions are fixed later, this indirection
    // can be removed and replaced by direct named output resolution.
    private const uint PCN_LIGHTS_ADDRESS = 29380;
    private const uint PCN_LIGHTS_ADDRESS_2 = 29384;
    private const uint CLP_ADDR_1 = 29248; 
    private const uint CLP_ADDR_2 = 29238;
    private const uint CLP_ADDR_3 = 29250;

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


    // --- DCS-BIOS controls (string outputs used as placeholders, addresses overridden) ---
    private DCSBIOSOutput? PCN_LIGHTS_REGISTER; 
    private DCSBIOSOutput? CLP_REGISTER_1; 
    private DCSBIOSOutput? CLP_REGISTER_2; 
    private DCSBIOSOutput? CLP_REGISTER_3; 
    private DCSBIOSOutput? PCN_LIGHTS_REGISTER_2;

    private DCSBIOSOutput? PCN_DISP_L;
    private DCSBIOSOutput? PCN_DISP_R;
    private DCSBIOSOutput? PCN_DISP_PREP;
    private DCSBIOSOutput? PCN_DISP_DEST;

    private string _pcnDispL = "N00.00.0";
    private string _pcnDispR = "E00.00.0";
    private string _pcnPrep = "P-1"; 
    private string _pcnDest = "D-1"; 
    
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

    public M2000C_Listener(ICdu? mcdu, UserOptions options) : base(mcdu, AircraftRegistry.M2000C, options)
    {
    }

    protected override void RegisterCduControls() { }
    protected override void RegisterFrontpanelControls() { }

    protected override void InitializeDcsBiosOutputs()
    {
        PCN_DISP_L = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_L");
        PCN_DISP_R = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_R");
        PCN_DISP_PREP = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_PREP");
        PCN_DISP_DEST = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_DEST");
        // Reuse an existing PCN display output object and repoint its address to the raw lights register.
        PCN_LIGHTS_REGISTER = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_L");
        if (PCN_LIGHTS_REGISTER != null) PCN_LIGHTS_REGISTER.Address = PCN_LIGHTS_ADDRESS;

        // Reuse PCN display outputs as generic DCSBIOSOutput instances for CLP bitfield registers.
        CLP_REGISTER_1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_R");
        if (CLP_REGISTER_1 != null) CLP_REGISTER_1.Address = CLP_ADDR_1;

        CLP_REGISTER_2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_PREP");
        if (CLP_REGISTER_2 != null) CLP_REGISTER_2.Address = CLP_ADDR_2;

        CLP_REGISTER_3 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_DEST");
        if (CLP_REGISTER_3 != null) CLP_REGISTER_3.Address = CLP_ADDR_3;

        PCN_LIGHTS_REGISTER_2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("PCN_DISP_DEST");
        if (PCN_LIGHTS_REGISTER_2 != null) PCN_LIGHTS_REGISTER_2.Address = PCN_LIGHTS_ADDRESS_2;
    }

    public override void DcsBiosDataReceived(object sender, DCSBIOSDataEventArgs e)
    {
        try
        {
            UpdateCounter(e.Address, e.Data);

            if (mcdu != null)
            { 
                bool refreshLeds = false;
                bool refreshDisplay = false;

                if (e.Address == PCN_LIGHTS_ADDRESS)
                {
                    ushort val = (ushort)e.Data;
                    mcdu.Leds.Fm1 = (val & MASK_ALN) > 0;
                    mcdu.Leds.Rdy = (val & MASK_PRET) > 0;
                    mcdu.Leds.Fm = (val & MASK_NDEG) > 0;
                    mcdu.Leds.Ind = (val & MASK_MIP) > 0;
                    mcdu.Leds.Fm2 = (val & MASK_SEC) > 0;
                    refreshLeds = true;
                }

                if (e.Address == PCN_LIGHTS_ADDRESS_2)
                {
                    ushort val = (ushort)e.Data;
                    mcdu.Leds.Fail = (val & MASK_UNI) > 0;
                    refreshLeds = true;
                }

                if (e.Address == CLP_ADDR_1) {
                    ushort val = (ushort)e.Data;
                    if (_clpValue1 != val) { _clpValue1 = val; refreshDisplay = true; }
                }
                if (e.Address == CLP_ADDR_2) {
                    ushort val = (ushort)e.Data;
                    if (_clpValue2 != val) { _clpValue2 = val; refreshDisplay = true; }
                }
                if (e.Address == CLP_ADDR_3) {
                    ushort val = (ushort)e.Data;
                    if (_clpValue3 != val) { _clpValue3 = val; refreshDisplay = true; }
                }

                if (refreshLeds) mcdu.RefreshLeds();
                if (refreshDisplay) UpdateCautionPanel();

            }
        }
        catch (Exception ex)
        {
            // Instrumentation: include raw data, address, current register snapshot
            App.Logger.Error(ex,
                "M2000C_Listener.DcsBiosDataReceived failed | addr=0x{Address:X4} data=0x{Data:X4} clp1=0x{Clp1:X4} clp2=0x{Clp2:X4} clp3=0x{Clp3:X4} leds=[Ind={Ind},Rdy={Rdy},Fail={Fail}]",
                new {
                    Address = e.Address,
                    Data = e.Data,
                    Clp1 = _clpValue1,
                    Clp2 = _clpValue2,
                    Clp3 = _clpValue3,
                    Ind = mcdu?.Leds?.Ind,
                    Rdy = mcdu?.Leds?.Rdy,
                    Fail = mcdu?.Leds?.Fail
                });
        }
    }

    public override void DCSBIOSStringReceived(object sender, DCSBIOSStringDataEventArgs e)
    {
        if (mcdu != null)
        {
            var output = GetCompositor(DEFAULT_PAGE);
            try
            {
                if (PCN_DISP_L != null && e.Address == PCN_DISP_L.Address)
                {
                    _pcnDispL = e.StringData; 
                    UpdateCombinedPCNDisplay(output);
                }
                else if (PCN_DISP_R != null && e.Address == PCN_DISP_R.Address)
                {
                    _pcnDispR = e.StringData; 
                    UpdateCombinedPCNDisplay(output);
                }
                else if (PCN_DISP_PREP != null && e.Address == PCN_DISP_PREP.Address)
                {
                    _pcnPrep = e.StringData;
                    UpdateCombinedPrepDestDisplay(output);
                }
                else if (PCN_DISP_DEST != null && e.Address == PCN_DISP_DEST.Address)
                {
                    _pcnDest = e.StringData;
                    UpdateCombinedPrepDestDisplay(output);
                }
            
                mcdu.RefreshDisplay();
            }
            catch (Exception ex)
            {
                App.Logger.Error(ex,
                    "Failed to process DCS-BIOS string data | addr=0x{Address:X4} len={Len} value='{Val}' pcnL='{PCNL}' pcnR='{PCNR}' prep='{Prep}' dest='{Dest}'",
                    new {
                        Address = e.Address,
                        Len = e.StringData?.Length ?? -1,
                        Val = e.StringData,
                        PCNL = _pcnDispL,
                        PCNR = _pcnDispR,
                        Prep = _pcnPrep,
                        Dest = _pcnDest
                    });
            }
        }
    }
    
    private void UpdateCautionPanel()
    {
        if (mcdu == null) return;
        
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
        if (mcdu == null) return;
        
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

        mcdu.RefreshDisplay();
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
}