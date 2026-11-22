using System.Globalization;

namespace WCtrlDcsBiosBridge.Aircrafts.UH1H;

/// <summary>The radio the intercom selector currently points the panel at.</summary>
internal enum UH1HRadio
{
    None,
    VhfAm,
    Uhf,
    Fm,
}

/// <summary>
/// Drives a two-display radio panel (the RMP32) on the UH-1H. The aircraft has no standby
/// frequency, so the standby is local to the bridge: it starts at the radio's current frequency
/// and the panel's rings tune it. SWAP sets the radio to the standby frequency through DCS-BIOS
/// and keeps the frequency it had as the new standby. Which radio is shown and tuned follows
/// the intercom mode selector (PVT - INT - FM - UHF - VHF AM), which keeps priority: the ADF
/// key shows the ADF in its place, and the VHF keys or a move of the selector bring the
/// communication radio back. While the ADF is shown the coarse ring turns its tuning knob and
/// the fine ring steps its three-position band switch, which stops at the ends rather than
/// wrapping round.
/// </summary>
internal sealed class UH1H_RmpRadio
{
    private sealed record Band(int MinKhz, int MaxKhz, int StepKhz);

    private static readonly Band VhfAmBand = new(116_000, 149_975, 25);
    private static readonly Band UhfBand = new(225_000, 399_950, 50);
    private static readonly Band FmBand = new(30_000, 75_950, 50);

    private const int CoarseKhz = 1_000;

    // One detent of the coarse ring on the ADF TUNE knob, out of its 65535 travel. The knob
    // takes relative steps (DCS-BIOS variable_step) and there is no frequency to aim for, so
    // this is a feel setting, not a derived value.
    private const int AdfTuneStep = 655;

    private const int AdfBandCount = 3;

    private readonly object _gate = new();
    private readonly UH1H_State _state;
    private readonly RadioPanelState _panel;

    private readonly Action<string, string> _send;
    private readonly Action<FlightDeckSignal, bool> _setSignal;

    private readonly Dictionary<UH1HRadio, int> _standbyKhz = new();

    private bool _showAdf;
    private int _lastIntMode;

    // The band asked for and not yet reported back. Detents arrive faster than DCS answers, so
    // stepping from the reported band would send the same step twice and miss the last position.
    private int? _pendingAdfBand;

    /// <param name="send">
    /// Sends a DCS-BIOS command: the control id and its argument. The "Radio Frequencies"
    /// controls (<c>VHF_ARC134</c>, <c>UHF_ARC51</c>, <c>VHF_ARC131</c>) take an absolute
    /// frequency as text.
    /// </param>
    /// <param name="setSignal">
    /// Lights the panel's key LEDs: which communication radio is selected, or the ADF.
    /// </param>
    public UH1H_RmpRadio(
        UH1H_State state,
        RadioPanelState panel,
        Action<string, string> send,
        Action<FlightDeckSignal, bool> setSignal)
    {
        _state = state;
        _panel = panel;
        _send = send;
        _setSignal = setSignal;
        _panel.KeyHandler = OnKey;
    }

    /// <summary>
    /// The radio selected on the intercom panel. The DCS-BIOS INT_MODE value is 0 PVT, 1 INT,
    /// then 2 VHF FM, 3 UHF and 4 VHF AM; 5 is not wired to a radio. PVT and INT select none.
    /// </summary>
    public static UH1HRadio RadioForIntMode(int intMode) => intMode switch
    {
        2 => UH1HRadio.Fm,
        3 => UH1HRadio.Uhf,
        4 => UH1HRadio.VhfAm,
        _ => UH1HRadio.None,
    };

    /// <summary>Recomputes the displays. Call after any cockpit value they depend on changes.</summary>
    public void Refresh()
    {
        lock (_gate) RefreshLocked();
    }

    private void RefreshLocked()
    {
        // The selector wins: any move of it takes the panel back to the communication radios.
        if (_state.IntMode != _lastIntMode)
        {
            _lastIntMode = _state.IntMode;
            _showAdf = false;
        }

        if (_showAdf)
        {
            if (_pendingAdfBand == (int)_state.adfBand) _pendingAdfBand = null;

            LightKeys(UH1HRadio.None, adf: true);
            ShowAdfLocked();
            return;
        }

        var radio = RadioForIntMode(_state.IntMode);
        LightKeys(radio, adf: false);
        if (radio == UH1HRadio.None)
        {
            _panel.Show(null, null);
            return;
        }

        var activeKhz = ActiveKhz(radio);
        var presetActive = radio == UH1HRadio.Uhf
            && _state.UhfFunction != UHFFunctionDial.OFF
            && _state.UhfMode == UHFModeDial.PRESET;

        // A radio that is off, or one the cockpit shows no frequency for, blanks the panel.
        if (activeKhz is null && !presetActive)
        {
            _panel.Show(null, null);
            return;
        }

        // In preset mode DCS-BIOS exports the channel number, not its frequency.
        var active = presetActive ? $"CH {_state.UhfPreset,3}" : Format(activeKhz);
        var standby = Format(StandbyKhz(radio, activeKhz));

        _panel.Show(active, standby);
    }

    /// <summary>
    /// Coarse ring: the ADF tuning knob, clockwise raising the frequency. Fine ring: the band
    /// switch, one position per detent and stopping at the first and last. The other keys have no
    /// meaning for the ADF, which has no standby.
    /// </summary>
    private void OnAdfKey(RadioPanelKey key)
    {
        switch (key)
        {
            case RadioPanelKey.CoarseUp:
                _send("ADF_TUNE", $"+{AdfTuneStep}");
                break;
            case RadioPanelKey.CoarseDown:
                _send("ADF_TUNE", $"-{AdfTuneStep}");
                break;
            case RadioPanelKey.FineUp:
                StepAdfBand(+1);
                break;
            case RadioPanelKey.FineDown:
                StepAdfBand(-1);
                break;
        }
    }

    private void StepAdfBand(int delta)
    {
        var current = _pendingAdfBand ?? (int)_state.adfBand;
        var next = Math.Clamp(current + delta, 0, AdfBandCount - 1);
        if (next == current) return;

        _pendingAdfBand = next;
        _send("ADF_BAND", next.ToString(CultureInfo.InvariantCulture));
    }

    /// <summary>
    /// VHF1 is the VHF AM radio, VHF2 the UHF and VHF3 the FM. The key of the radio the
    /// selector points at is lit, even when that radio is off: it is the selection that shows.
    /// </summary>
    private void LightKeys(UH1HRadio radio, bool adf)
    {
        _setSignal(FlightDeckSignal.RadioSlot1, radio == UH1HRadio.VhfAm);
        _setSignal(FlightDeckSignal.RadioSlot2, radio == UH1HRadio.Uhf);
        _setSignal(FlightDeckSignal.RadioSlot3, radio == UH1HRadio.Fm);
        _setSignal(FlightDeckSignal.RadioAdf, adf);
    }

    /// <summary>
    /// The ADF frequency in kHz on the left and the mode on the right (ADF, ANT or LOOP). The ADF
    /// dial can only be read, so there is no standby. An ADF that is off blanks the panel.
    /// </summary>
    private void ShowAdfLocked()
    {
        if (_state.adfMode == ADFMode.OFF)
        {
            _panel.Show(null, null);
            return;
        }

        var mode = _state.adfMode switch
        {
            ADFMode.ADF => "ADF",
            ADFMode.ANT => "ANT",
            _ => "LOOP",
        };

        _panel.Show(((int)_state.AdfFrequency).ToString(CultureInfo.InvariantCulture), mode);
    }

    private void OnKey(RadioPanelKey key)
    {
        lock (_gate)
        {
            switch (key)
            {
                case RadioPanelKey.ShowAdf:
                    _showAdf = true;
                    _pendingAdfBand = null;
                    RefreshLocked();
                    return;
                case RadioPanelKey.ShowComm:
                    _showAdf = false;
                    _pendingAdfBand = null;
                    RefreshLocked();
                    return;
            }

            if (_showAdf)
            {
                OnAdfKey(key);
                return;
            }

            var radio = RadioForIntMode(_state.IntMode);
            if (radio == UH1HRadio.None) return;

            var band = BandOf(radio);
            var activeKhz = ActiveKhz(radio);

            switch (key)
            {
                case RadioPanelKey.Swap:
                    Swap(radio, band, activeKhz);
                    break;
                case RadioPanelKey.Sync:
                    if (activeKhz is int khz) _standbyKhz[radio] = Snap(band, khz);
                    break;
                case RadioPanelKey.CoarseUp:
                    Tune(radio, band, activeKhz, +CoarseKhz);
                    break;
                case RadioPanelKey.CoarseDown:
                    Tune(radio, band, activeKhz, -CoarseKhz);
                    break;
                case RadioPanelKey.FineUp:
                    Tune(radio, band, activeKhz, +band.StepKhz);
                    break;
                case RadioPanelKey.FineDown:
                    Tune(radio, band, activeKhz, -band.StepKhz);
                    break;
            }

            RefreshLocked();
        }
    }

    /// <summary>
    /// Sets the radio to the standby and keeps what it was on as the new standby. Does nothing
    /// when the radio is off or on a preset: its frequency is unknown, and there is nothing to
    /// keep as the standby.
    /// </summary>
    private void Swap(UH1HRadio radio, Band band, int? activeKhz)
    {
        if (activeKhz is not int active) return;

        var target = StandbyKhz(radio, activeKhz) ?? active;
        _standbyKhz[radio] = Snap(band, active);
        if (target == active) return;

        _send(ControlOf(radio), Format(radio, target));
    }

    private static string ControlOf(UH1HRadio radio) => radio switch
    {
        UH1HRadio.Uhf => "UHF_ARC51",
        UH1HRadio.Fm => "VHF_ARC131",
        _ => "VHF_ARC134",
    };

    private void Tune(UH1HRadio radio, Band band, int? activeKhz, int deltaKhz)
    {
        var current = StandbyKhz(radio, activeKhz) ?? band.MinKhz;
        _standbyKhz[radio] = Snap(band, current + deltaKhz);
    }

    /// <summary>The standby the user tuned, or the radio's own frequency until they do.</summary>
    private int? StandbyKhz(UH1HRadio radio, int? activeKhz)
    {
        if (_standbyKhz.TryGetValue(radio, out var khz)) return khz;
        return activeKhz is int a ? Snap(BandOf(radio), a) : null;
    }

    private int? ActiveKhz(UH1HRadio radio) => radio switch
    {
        UH1HRadio.VhfAm when _state.VhfPwr => ParseMhz(_state.VhfFreq),
        UH1HRadio.Uhf when _state.UhfFunction != UHFFunctionDial.OFF && _state.UhfMode != UHFModeDial.PRESET => ParseMhz(_state.UhfFreq),
        UH1HRadio.Fm when _state.FmMode != VhfFmMode.OFF => ParseMhz(_state.FmFreq),
        _ => null,
    };

    private static Band BandOf(UH1HRadio radio) => radio switch
    {
        UH1HRadio.Uhf => UhfBand,
        UH1HRadio.Fm => FmBand,
        _ => VhfAmBand,
    };

    private static int Snap(Band band, int khz)
    {
        var clamped = Math.Clamp(khz, band.MinKhz, band.MaxKhz);
        var steps = (int)Math.Round((clamped - band.MinKhz) / (double)band.StepKhz);
        return Math.Min(band.MinKhz + steps * band.StepKhz, band.MaxKhz);
    }

    /// <summary>"123.450" to 123450, or null when the text is not a frequency (DCS shows dashes while off).</summary>
    private static int? ParseMhz(string? text)
    {
        if (string.IsNullOrWhiteSpace(text)) return null;
        return double.TryParse(text.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out var mhz) && mhz > 0
            ? (int)Math.Round(mhz * 1000)
            : null;
    }

    /// <summary>
    /// The text the radio's DCS-BIOS control takes. The FM radio's display has two decimals
    /// (51.75), the others three.
    /// </summary>
    private static string Format(UH1HRadio radio, int khz) => radio == UH1HRadio.Fm
        ? $"{khz / 1000}.{khz % 1000 / 10:00}"
        : Format(khz)!;

    /// <summary>123450 to "123.450": the decimal point rides on the digit before it.</summary>
    private static string Format(int? khz) =>
        khz is int k ? $"{k / 1000}.{k % 1000:000}" : string.Empty;
}
