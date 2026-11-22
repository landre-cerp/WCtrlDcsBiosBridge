using DCS_BIOS;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.UH1H;

internal class UH1H_Listener : AircraftListener
{
    private readonly UH1H_State _state = new();
    private UH1H_RmpRadio? _rmpRadio;

    private const int LINE_NAV = 2;
    private const int LINE_VHF  = 3;
    private const int LINE_UHF  = 4;
    private const int LINE_FM   = 5;

    private const int LINE_ADF  = 7;
    private const int LINE_IFF1 = 9;
    private const int LINE_IFF2 = 10;
    private const int LINE_IFF3 = 11;

    public UH1H_Listener(UserOptions options) : base(AircraftRegistry.UH1H, options) { }

    protected override void InitializeDcsBiosOutputs() => InitializeDisplay();

    // No CDU exists in the real aircraft — brightness starts at 100% and is
    // managed by the physical BRT/DIM keys on the CDU hardware.
    protected override void InitMcduBrightness()
    {
        if (cdu == null) return;
        SetCduBacklightBrightnessPercent(100);
        SetCduLedBrightnessPercent(100);
        SetCduDisplayBrightnessPercent(100);
    }

    // The RMP32 radio panel. These handlers feed the same state as the CDU radio page below
    // (so both can run together, and the page's handlers only exist when a CDU does), then
    // ask the panel logic to recompute its displays.
    protected override void RegisterFrontpanelControls()
    {
        var panel = new RadioPanelState();
        _rmpRadio = new UH1H_RmpRadio(
            _state,
            panel,
            (control, value) => DCSBIOS.Send("UH1H_RMP", $"{control} {value}\n"),
            FlightDeck.SetSignal);
        FlightDeck.RadioPanel = panel;

        void Watch(Action update) { update(); _rmpRadio.Refresh(); }

        RegisterUInt("INT_MODE", v => Watch(() => _state.IntMode = (int)v));

        RegisterUInt("VHFCOMM_PWR", v => Watch(() => _state.VhfPwr = v != 0));
        RegisterStr("VHFCOMM_FREQ", v => Watch(() => _state.VhfFreq = v));

        RegisterStr("UHF_FREQ", v => Watch(() => _state.UhfFreq = v));
        RegisterUInt("UHF_PRESET", v => Watch(() => _state.UhfPreset = (int)v + 1));
        RegisterUInt("UHF_MODE", v => Watch(() => _state.UhfMode = (UHFModeDial)v));
        RegisterUInt("UHF_FUNCTION", v => Watch(() => _state.UhfFunction = (UHFFunctionDial)v));

        RegisterUInt("VHFFM_FREQ1", v => Watch(() => _state.FmD1 = (int)v));
        RegisterUInt("VHFFM_FREQ2", v => Watch(() => _state.FmD2 = (int)v));
        RegisterUInt("VHFFM_FREQ3", v => Watch(() => _state.FmD3 = (int)v));
        RegisterUInt("VHFFM_FREQ4", v => Watch(() => _state.FmD4 = (int)v));
        RegisterUInt("VHFFM_MODE", v => Watch(() => _state.FmMode = (VhfFmMode)v));

        RegisterUInt("ADF_MODE", v => Watch(() => _state.adfMode = (ADFMode)v));
        RegisterUInt("ADF_BAND", v => Watch(() => _state.adfBand = (ADFBand)v));
        RegisterUInt("ADF_FREQ", v => Watch(() => _state.adfFreqPos = (int)v));

        _rmpRadio.Refresh();
    }

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Brt)
        {
            _state.BrightnessPercent = Math.Min(100, _state.BrightnessPercent + UH1H_State.BrtStep);
            ApplyBrightness();
        }
        else if (e.Key == Key.Dim)
        {
            _state.BrightnessPercent = Math.Max(0, _state.BrightnessPercent - UH1H_State.BrtStep);
            ApplyBrightness();
        }
    }

    private void ApplyBrightness()
    {
        SetCduBacklightBrightnessPercent(_state.BrightnessPercent);
        SetCduDisplayBrightnessPercent(_state.BrightnessPercent);
        SetCduLedBrightnessPercent(_state.BrightnessPercent);
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        // The master caution is declared in LedDefaults.

        // ── VHF COMM (ARC-134) ────────────────────────────────────────────
        RegisterUInt("VHFCOMM_PWR", v =>
        {
            _state.VhfPwr = v != 0;
            UpdateVhfLine();
        });
        RegisterStr("VHFCOMM_FREQ", v =>
        {
            _state.VhfFreq = v;
            UpdateVhfLine();
        });

        // ── UHF (ARC-51) ─────────────────────────────────────────────────
        RegisterStr("UHF_FREQ", v =>
        {
            _state.UhfFreq = v;
            UpdateUhfLine();
        });
        RegisterUInt("UHF_PRESET", v =>
        {
            _state.UhfPreset = (int)v + 1;
            UpdateUhfLine();
        });
        RegisterUInt("UHF_MODE", v =>
        {
            _state.UhfMode = (UHFModeDial)v;
            UpdateUhfLine();
        });
        RegisterUInt("UHF_FUNCTION", v =>
        {
            _state.UhfFunction = (UHFFunctionDial)v;
            UpdateUhfLine();
        });

        // ── VHF FM (ARC-131) ─────────────────────────────────────────────
        RegisterUInt("VHFFM_FREQ1", v => { _state.FmD1 = (int)v; UpdateFmLine(); });
        RegisterUInt("VHFFM_FREQ2", v => { _state.FmD2 = (int)v; UpdateFmLine(); });
        RegisterUInt("VHFFM_FREQ3", v => { _state.FmD3 = (int)v; UpdateFmLine(); });
        RegisterUInt("VHFFM_FREQ4", v => { _state.FmD4 = (int)v; UpdateFmLine(); });
        RegisterUInt("VHFFM_MODE",  v => { _state.FmMode = (VhfFmMode)v; UpdateFmLine(); });

        // ── VHF NAV (ARN-82) ─────────────────────────────────────────────
        RegisterUInt("VHFNAV_PWR", v =>
        {
            _state.NavPwr = v != 0;
            UpdateNavLine();
        });
        RegisterStr("VHFNAV_FREQ", v =>
        {
            _state.NavFreq = v;
            UpdateNavLine();
        });

        // ── ADF (ARN-83) ─────────────────────────────────────────────────────
        // ADF_FREQ is the position of the tuning dial, not a frequency: UH1H_AdfScale turns it
        // into kHz, band by band.
        RegisterUInt("ADF_MODE", v => { _state.adfMode = (ADFMode)v; UpdateAdfLine(); });
        RegisterUInt("ADF_BAND", v => { _state.adfBand = (ADFBand)v; UpdateAdfLine(); });
        RegisterUInt("ADF_FREQ", v => { _state.adfFreqPos = (int)v; UpdateAdfLine(); });

        // ── IFF (AN/APX-72) — re-enabled after local DCS-BIOS patch ─────────────
        RegisterUInt("IFF_MASTER", v => { _state.IffMaster = (IffMaster)v; UpdateIffLines(); });
        RegisterUInt("IFF_CODE", v => { _state.IffCode = (IffCode)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE1_WHEEL1", v => { _state.IffM1W1 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE1_WHEEL2", v => { _state.IffM1W2 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE3A_WHEEL1", v => { _state.IffM3aW1 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE3A_WHEEL2", v => { _state.IffM3aW2 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE3A_WHEEL3", v => { _state.IffM3aW3 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_MODE3A_WHEEL4", v => { _state.IffM3aW4 = (int)v; UpdateIffLines(); });
        RegisterUInt("IFF_REPLY_IND", v => { _state.IffReply = v != 0; UpdateIffLines(); });
        RegisterUInt("IFF_TEST_IND", v => { _state.IffTest = v != 0; UpdateIffLines(); });
    }

    // ── Display update helpers ────────────────────────────────────────────

    private void UpdateVhfLine()
    {
        var comp = GetCompositor(DEFAULT_PAGE).Line(LINE_VHF).White().Write("VHF AM  ");
        if (!_state.VhfPwr)
            comp.Red().WriteLine("OFF".PadRight(7));
        else
            comp.Green().WriteLine(_state.VhfFreq.Trim().PadRight(7));
    }

    private void UpdateUhfLine()
    {
        var freqOrOff = _state.UhfFunction == UHFFunctionDial.OFF
            ? "OFF   "
            : _state.UhfMode == UHFModeDial.PRESET
                ? $"CH:{_state.UhfPreset:D2}  "
                : _state.UhfFreq.Trim().PadRight(6);

        var modeLabel = _state.UhfFunction switch
        {
            UHFFunctionDial.TR   => "T/R  ",
            UHFFunctionDial.TR_G => "T/R+G",
            UHFFunctionDial.DF   => "ADF  ",
            _                    => "     ",
        };

        var uhfOff = _state.UhfFunction == UHFFunctionDial.OFF;
        var uhfComp = GetCompositor(DEFAULT_PAGE).Line(LINE_UHF).White().Write("UHF     ");
        if (uhfOff) uhfComp.Red().Write(freqOrOff); else uhfComp.Green().Write(freqOrOff);
        uhfComp.White().WriteLine($" {modeLabel}");
    }

    private void UpdateFmLine()
    {
        var modeLabel = _state.FmMode switch
        {
            VhfFmMode.TR     => "T/R   ",
            VhfFmMode.RETRAN => "RETRAN",
            VhfFmMode.HOME   => "HOME  ",
            _                => "      ",
        };

        var fmOff = _state.FmMode == VhfFmMode.OFF;
        var freqOrOff = fmOff ? "OFF   " : _state.FmFreq.PadRight(6);

        var fmComp = GetCompositor(DEFAULT_PAGE).Line(LINE_FM).White().Write("VHF FM  ");
        if (fmOff) fmComp.Red().Write(freqOrOff); else fmComp.Green().Write(freqOrOff);
        fmComp.White().WriteLine($" {modeLabel}");
    }

    private void UpdateNavLine()
    {
        var comp = GetCompositor(DEFAULT_PAGE).Line(LINE_NAV).White().Write("VHF NAV ");
        if (!_state.NavPwr)
            comp.Red().WriteLine("OFF".PadRight(6));
        else
            comp.Green().WriteLine(_state.NavFreq.Trim().PadRight(6));
    }

    private void UpdateAdfLine()
    {
        var modeLabel = _state.adfMode switch
        {
            ADFMode.ADF  => "ADF ",
            ADFMode.ANT  => "ANT ",
            ADFMode.LOOP => "LOOP",
            _            => "    ",
        };

        var off = _state.adfMode == ADFMode.OFF;
        var freqOrOff = off ? "OFF   " : $"{(int)_state.AdfFrequency,4}  ";

        var comp = GetCompositor(DEFAULT_PAGE).Line(LINE_ADF).White().Write("ADF     ");
        if (off) comp.Red().Write(freqOrOff); else comp.Green().Write(freqOrOff);
        comp.White().WriteLine($" {modeLabel}");
    }

    private void UpdateIffLines()
    {
        var masterLabel = _state.IffMaster switch
        {
            IffMaster.LOW   => "LOW  ",
            IffMaster.NORM  => "NORM ",
            IffMaster.EMERG => "EMERG",
            IffMaster.OFF   => "OFF  ",
            _               => "STBY ",
        };
        var codeLabel = _state.IffCode switch
        {
            IffCode.B    => "B   ",
            IffCode.A    => "A   ",
            IffCode.HOLD => "HOLD",
            _            => "ZERO",
        };
        var iffOff = _state.IffMaster == IffMaster.OFF;
        var iffComp = GetCompositor(DEFAULT_PAGE).Line(LINE_IFF1).Amber().Write("IFF  ");
        if (iffOff) iffComp.Red().Write(masterLabel); else iffComp.Green().Write(masterLabel);
        iffComp.White().Write(" CODE:").Green().WriteLine(codeLabel);

        GetCompositor(DEFAULT_PAGE).Line(LINE_IFF2)
            .White().Write("M1:")
            .Green().Write(_state.IffM1Code)
            .White().Write("  M3A:")
            .Green().WriteLine(_state.IffM3aCode);

        var replyMark = _state.IffReply ? "⬡" : " ";
        var testMark  = _state.IffTest  ? "⬡" : " ";
        GetCompositor(DEFAULT_PAGE).Line(LINE_IFF3)
            .White().Write("REPLY:")
            .Green().Write(replyMark)
            .White().Write("  TEST:")
            .Green().WriteLine(testMark);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && CduDevice != null)
            CduDevice.KeyDown -= HandleKeyDown;
        base.Dispose(disposing);
    }

    private void InitializeDisplay()
    {
        if (!HasCdu) return;

        var comp = GetCompositor(DEFAULT_PAGE);
        comp.Clear()
            .Amber().Line(0).Centered("UH-1H HUEY RADIO");

        UpdateVhfLine();
        UpdateUhfLine();
        UpdateFmLine();
        UpdateNavLine();
        UpdateAdfLine();
        UpdateIffLines();
    }
}
