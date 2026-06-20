namespace WCtrlDcsBiosBridge.Aircrafts.UH1H;

internal enum ADFMode   { OFF = 0, ADF = 1, ANT = 2, LOOP = 3 }
internal enum ADFBand   { Low = 0, Mid = 1, High = 2 }

internal enum UHFFunctionDial { OFF = 0, TR = 1, TR_G = 2, DF = 3 }
internal enum UHFModeDial     { PRESET = 0, MANUAL = 1, GUARD = 2 }

internal enum VhfFmMode { OFF = 0, TR = 1, RETRAN = 2, HOME = 3 }

internal enum IffMaster { STBY = 1, LOW = 2, NORM = 3, EMERG = 4, OFF = 0 }
internal enum IffCode   { ZERO = 0, B = 1, A = 2, HOLD = 3 }

internal class UH1H_State
{
    // CDU brightness (no cockpit knob — managed by physical BRT/DIM keys)
    public const int BrtStep = 5;
    public int BrightnessPercent = 100;

    // VHF COMM (ARC-134)
    public string VhfFreq  = "-------";
    public bool   VhfPwr   = true;

    // UHF (ARC-51)
    public string UhfFreq     = "------";
    public int    UhfPreset   = 1;
    public UHFModeDial     UhfMode     = UHFModeDial.MANUAL;
    public UHFFunctionDial UhfFunction = UHFFunctionDial.OFF;

    // VHF FM (ARC-131) — four dial digits
    public int FmD1 = 0;   // tens of MHz:  v+3 → 3..7
    public int FmD2 = 0;   // units MHz:    0..9
    public int FmD3 = 0;   // tenths MHz:   0..9
    public int FmD4 = 0;   // 50kHz step:   0→'0', 1→'5'
    public VhfFmMode FmMode = VhfFmMode.OFF;

    public string FmFreq =>
        $"{FmD1 + 3}{FmD2}.{FmD3}{(FmD4 == 0 ? '0' : '5')}";

    // VHF NAV (ARN-82)
    public string NavFreq  = "------";
    public bool   NavPwr   = false;

    // ADF (ARN-83)
    public ADFMode adfMode = ADFMode.OFF;
    public ADFBand adfBand = ADFBand.Low;
    public int     adfFreqPos = 0;

    private static readonly (double Min, double Max)[] BandRanges =
    [
        (190, 400),
        (400, 850),
        (850, 1800),
    ];

    public double AdfFrequency
    {
        get
        {
            var (min, max) = BandRanges[(int)adfBand];
            return min + (max - min) * adfFreqPos / 65535.0;
        }
    }

    // IFF
    public IffMaster IffMaster = IffMaster.STBY;
    public IffCode   IffCode   = IffCode.ZERO;
    public int IffM1W1  = 0;
    public int IffM1W2  = 0;
    public int IffM3aW1 = 0;
    public int IffM3aW2 = 0;
    public int IffM3aW3 = 0;
    public int IffM3aW4 = 0;
    public bool IffReply = false;
    public bool IffTest  = false;

    // Mode 1 is 5-bit: high octal digit (0-3) = W2, low octal digit (0-7) = W1
    public string IffM1Code  => $"{IffM1W1}{IffM1W2}";
    public string IffM3aCode => $"{IffM3aW1}{IffM3aW2}{IffM3aW3}{IffM3aW4}";
}
