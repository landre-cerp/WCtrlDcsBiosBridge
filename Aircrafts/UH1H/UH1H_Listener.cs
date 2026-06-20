using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts.UH1H;

internal class UH1H_Listener : AircraftListener
{
    private readonly UH1H_State _state = new();

    private const int LINE_NAV = 2;
    private const int LINE_VHF  = 3;
    private const int LINE_UHF  = 4;
    private const int LINE_FM   = 5;
    
    // ADF disabled — ADF_FREQ output is bugged in DCS-BIOS for the UH-1H
    // private const int LINE_ADF  = 7;
    // IFF disabled — IFF_MODE*_WHEEL* outputs are bugged in DCS-BIOS for the UH-1H
    // private const int LINE_IFF1 = 9;
    // private const int LINE_IFF2 = 10;
    // private const int LINE_IFF3 = 11;

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

    protected override void RegisterFrontpanelControls() { }

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

        RegisterLight("MASTER_CAUTION_IND", v => SetCduLeds(fail: v == 1));

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

        // ── ADF — disabled: ADF_FREQ output is bugged in DCS-BIOS ───────────
        // RegisterUInt("ADF_MODE", v => { _state.adfMode = (ADFMode)v; UpdateAdfLine(); });
        // RegisterUInt("ADF_BAND", v => { _state.adfBand = (ADFBand)v; UpdateAdfLine(); });
        // RegisterUInt("ADF_FREQ", v => { _state.adfFreqPos = (int)v;  UpdateAdfLine(); });

        // ── IFF — disabled: IFF_MODE*_WHEEL* outputs are bugged in DCS-BIOS ────
        // RegisterUInt("IFF_MASTER",        v => { _state.IffMaster = (IffMaster)v; UpdateIffLines(); });
        // RegisterUInt("IFF_CODE",          v => { _state.IffCode   = (IffCode)v;   UpdateIffLines(); });
        // RegisterUInt("IFF_MODE1_WHEEL1",  v => { _state.IffM1W1   = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_MODE1_WHEEL2",  v => { _state.IffM1W2   = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_MODE3A_WHEEL1", v => { _state.IffM3aW1  = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_MODE3A_WHEEL2", v => { _state.IffM3aW2  = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_MODE3A_WHEEL3", v => { _state.IffM3aW3  = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_MODE3A_WHEEL4", v => { _state.IffM3aW4  = (int)v;       UpdateIffLines(); });
        // RegisterUInt("IFF_REPLY_IND",     v => { _state.IffReply  = v != 0;        UpdateIffLines(); });
        // RegisterUInt("IFF_TEST_IND",      v => { _state.IffTest   = v != 0;        UpdateIffLines(); });
    }

    // ── Display update helpers ────────────────────────────────────────────

    private void UpdateVhfLine()
    {
        var comp = GetCompositor(DEFAULT_PAGE).Line(LINE_VHF).White().Write("VHF AM  ");
        if (!_state.VhfPwr)
            comp.Green().WriteLine("OFF");
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

        GetCompositor(DEFAULT_PAGE).Line(LINE_UHF)
            .White().Write("UHF     ")
            .Green().Write(freqOrOff)
            .White().WriteLine($" {modeLabel}");
    }

    private void UpdateFmLine()
    {
        var modeLabel = _state.FmMode switch
        {
            VhfFmMode.TR     => "T/R   ",
            VhfFmMode.RETRAN => "RETRAN",
            VhfFmMode.HOME   => "HOME  ",
            _                => "OFF   ",
        };

        var freqOrOff = _state.FmMode == VhfFmMode.OFF ? "OFF   " : _state.FmFreq.PadRight(6);

        GetCompositor(DEFAULT_PAGE).Line(LINE_FM)
            .White().Write("VHF FM  ")
            .Green().Write(freqOrOff)
            .White().WriteLine($" {modeLabel}");
    }

    private void UpdateNavLine()
    {
        var comp = GetCompositor(DEFAULT_PAGE).Line(LINE_NAV).White().Write("VHF NAV ");
        if (!_state.NavPwr)
            comp.Green().WriteLine("OFF");
        else
            comp.Green().WriteLine(_state.NavFreq.Trim().PadRight(6));
    }

    // ADF disabled — ADF_FREQ output is bugged in DCS-BIOS for the UH-1H
    // private void UpdateAdfLine() { ... }

    // IFF disabled — IFF_MODE*_WHEEL* outputs are bugged in DCS-BIOS for the UH-1H
    // private void UpdateIffLines() { ... }

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
        // UpdateAdfLine();  — disabled, see ADF note above
        // UpdateIffLines(); — disabled, see IFF note above
    }
}
