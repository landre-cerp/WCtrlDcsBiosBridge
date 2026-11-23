using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Aircrafts.UH1H;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class UH1HRmpRadioTests
{
    // INT_MODE positions, as DCS-BIOS exports them.
    private const int Pvt = 0, Int = 1, Fm = 2, Uhf = 3, VhfAm = 4, Unused = 5;

    private static (UH1H_State State, RadioPanelState Panel, UH1H_RmpRadio Radio, List<(string Control, string Value)> Sent) Create() =>
        Create(new FlightDeckState());

    private static (UH1H_State State, RadioPanelState Panel, UH1H_RmpRadio Radio, List<(string Control, string Value)> Sent) Create(FlightDeckState deck)
    {
        var sent = new List<(string Control, string Value)>();
        var state = new UH1H_State { IntMode = VhfAm, VhfFreq = "123.450", VhfPwr = true };
        var panel = new RadioPanelState();
        var radio = new UH1H_RmpRadio(state, panel, (control, value) => sent.Add((control, value)), deck.SetSignal);
        radio.Refresh();
        return (state, panel, radio, sent);
    }

    [Theory]
    [InlineData(Pvt, 0)]
    [InlineData(Int, 0)]
    [InlineData(VhfAm, 1)]
    [InlineData(Uhf, 2)]
    [InlineData(Fm, 3)]
    [InlineData(Unused, 0)]
    public void IntModeSelectsTheRadio(int intMode, int expected) =>
        Assert.Equal((UH1HRadio)expected, UH1H_RmpRadio.RadioForIntMode(intMode));

    [Fact]
    public void StandbyStartsAtTheActiveFrequency()
    {
        var (_, panel, _, _) = Create();

        Assert.Equal("123.450", panel.Active);
        Assert.Equal("123.450", panel.Standby);
    }

    [Fact]
    public void RingsTuneTheStandbyAndLeaveTheActiveAndTheSimAlone()
    {
        var (_, panel, _, sent) = Create();

        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.FineUp);

        Assert.Equal("123.450", panel.Active);
        Assert.Equal("124.475", panel.Standby);
        Assert.Empty(sent);
    }

    [Fact]
    public void SwapSetsTheRadioToTheStandbyAndKeepsTheOldActiveAsStandby()
    {
        var (_, panel, _, sent) = Create();
        panel.Press(RadioPanelKey.CoarseUp);                // standby 124.450

        panel.Press(RadioPanelKey.Swap);

        Assert.Equal(("VHF_ARC134", "124.450"), Assert.Single(sent));
        Assert.Equal("123.450", panel.Standby);
    }

    [Fact]
    public void ActiveFollowsTheCockpitOnceTheSimAnswers()
    {
        var (state, panel, radio, _) = Create();
        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.Swap);

        state.VhfFreq = "124.450";                          // what DCS-BIOS reports back
        radio.Refresh();

        Assert.Equal("124.450", panel.Active);
        Assert.Equal("123.450", panel.Standby);
    }

    [Fact]
    public void SwappingBackRestoresTheFirstFrequency()
    {
        var (state, panel, radio, sent) = Create();
        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.Swap);
        state.VhfFreq = "124.450";
        radio.Refresh();

        panel.Press(RadioPanelKey.Swap);

        Assert.Equal(("VHF_ARC134", "123.450"), sent[^1]);
        Assert.Equal("124.450", panel.Standby);
    }

    [Fact]
    public void SwapWithNothingTunedSendsNothing()
    {
        var (_, panel, _, sent) = Create();

        panel.Press(RadioPanelKey.Swap);

        Assert.Empty(sent);
    }

    [Fact]
    public void KnobPushCopiesTheActiveIntoTheStandby()
    {
        var (_, panel, _, sent) = Create();
        panel.Press(RadioPanelKey.CoarseUp);

        panel.Press(RadioPanelKey.Sync);

        Assert.Equal("123.450", panel.Standby);
        Assert.Empty(sent);
    }

    [Fact]
    public void TuningStopsAtTheEdgesOfTheBand()
    {
        var (_, panel, _, _) = Create();

        for (var i = 0; i < 40; i++) panel.Press(RadioPanelKey.CoarseUp);
        Assert.Equal("149.975", panel.Standby);

        for (var i = 0; i < 40; i++) panel.Press(RadioPanelKey.CoarseDown);
        Assert.Equal("116.000", panel.Standby);

        Assert.Equal("123.450", panel.Active);
    }

    [Fact]
    public void EachRadioKeepsItsOwnStandbyAndSwapsOnItsOwnControl()
    {
        var (state, panel, radio, sent) = Create();
        state.UhfFunction = UHFFunctionDial.TR;
        state.UhfMode = UHFModeDial.MANUAL;
        state.UhfFreq = "251.000";

        panel.Press(RadioPanelKey.CoarseUp);                // VHF AM standby: 124.450
        state.IntMode = Uhf;
        radio.Refresh();

        Assert.Equal("251.000", panel.Active);
        Assert.Equal("251.000", panel.Standby);

        panel.Press(RadioPanelKey.FineUp);                  // UHF standby: 251.050
        panel.Press(RadioPanelKey.Swap);
        Assert.Equal(("UHF_ARC51", "251.050"), Assert.Single(sent));

        state.IntMode = VhfAm;
        radio.Refresh();
        Assert.Equal("124.450", panel.Standby);
    }

    [Fact]
    public void FmFrequencyComesFromTheFourDialsAndIsSentWithTwoDecimals()
    {
        var (state, panel, radio, sent) = Create();
        state.IntMode = Fm;
        state.FmMode = VhfFmMode.TR;
        state.FmD1 = 2;   // 5x MHz
        state.FmD2 = 1;
        state.FmD3 = 7;
        state.FmD4 = 1;   // +50 kHz
        radio.Refresh();

        Assert.Equal("51.750", panel.Active);

        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.Swap);

        Assert.Equal(("VHF_ARC131", "52.75"), Assert.Single(sent));
    }

    [Fact]
    public void UhfPresetShowsTheChannelNumberAndCannotBeSwapped()
    {
        var (state, panel, radio, sent) = Create();
        state.IntMode = Uhf;
        state.UhfFunction = UHFFunctionDial.TR;
        state.UhfMode = UHFModeDial.PRESET;
        state.UhfPreset = 7;
        radio.Refresh();

        Assert.Equal("CH   7", panel.Active);

        panel.Press(RadioPanelKey.Swap);
        Assert.Empty(sent);
    }

    [Theory]
    [InlineData(Pvt)]
    [InlineData(Int)]
    [InlineData(Unused)]
    public void PanelIsBlankWhenNoRadioIsSelected(int intMode)
    {
        var (state, panel, radio, _) = Create();
        state.IntMode = intMode;
        radio.Refresh();

        Assert.Equal(string.Empty, panel.Active);
        Assert.Equal(string.Empty, panel.Standby);
    }

    [Fact]
    public void PanelIsBlankWhenTheSelectedRadioIsOff()
    {
        var (state, panel, radio, sent) = Create();
        state.VhfPwr = false;
        radio.Refresh();

        Assert.Equal(string.Empty, panel.Active);
        Assert.Equal(string.Empty, panel.Standby);

        panel.Press(RadioPanelKey.Swap);
        Assert.Empty(sent);
    }

    private static void TuneAdf(UH1H_State state, ADFMode mode)
    {
        state.adfMode = mode;
        state.adfBand = ADFBand.Mid;
        state.adfFreqPos = 12222;                           // 443 kHz on the mid band (beacon checked in the simulator)
    }

    [Fact]
    public void AdfKeyShowsTheFrequencyAndTheModeWithoutAStandby()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.ANT);
        radio.Refresh();

        panel.Press(RadioPanelKey.ShowAdf);

        Assert.Equal("443", panel.Active);
        Assert.Equal("ANT", panel.Standby);
    }

    [Fact]
    public void AdfFollowsTheDialWhileShown()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.ADF);
        panel.Press(RadioPanelKey.ShowAdf);

        state.adfBand = ADFBand.High;
        state.adfFreqPos = 9544;                            // 950 kHz on the high band
        radio.Refresh();

        Assert.Equal("950", panel.Active);
    }

    [Fact]
    public void AdfThatIsOffBlanksThePanel()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.OFF);
        radio.Refresh();

        panel.Press(RadioPanelKey.ShowAdf);

        Assert.Equal(string.Empty, panel.Active);
        Assert.Equal(string.Empty, panel.Standby);
    }

    [Fact]
    public void SwapAndSyncAreInertWhileTheAdfIsShown()
    {
        var (state, panel, radio, sent) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        panel.Press(RadioPanelKey.Swap);
        panel.Press(RadioPanelKey.Sync);

        Assert.Empty(sent);
        Assert.Equal("443", panel.Active);

        panel.Press(RadioPanelKey.ShowComm);                // the VHF AM standby was never touched
        Assert.Equal("123.450", panel.Standby);
    }

    [Fact]
    public void CoarseRingTurnsTheAdfTuningKnob()
    {
        var (state, panel, radio, sent) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.CoarseDown);

        // A positive ADF_TUNE step lowers the UH-1H's frequency, so raising it sends a negative one.
        Assert.Equal(new[] { ("ADF_TUNE", "-200"), ("ADF_TUNE", "+200") }, sent);
    }

    [Fact]
    public void FineRingStepsTheBandAndStopsAtBothEnds()
    {
        var (state, panel, radio, sent) = Create();
        TuneAdf(state, ADFMode.ADF);                        // mid band, position 1
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        panel.Press(RadioPanelKey.FineUp);                  // -> 2
        panel.Press(RadioPanelKey.FineUp);                  // already last: nothing
        Assert.Equal(("ADF_BAND", "2"), Assert.Single(sent));

        state.adfBand = ADFBand.High;                       // DCS reports it
        radio.Refresh();
        sent.Clear();

        panel.Press(RadioPanelKey.FineDown);                // -> 1
        panel.Press(RadioPanelKey.FineDown);                // -> 0
        panel.Press(RadioPanelKey.FineDown);                // already first: nothing
        Assert.Equal(new[] { ("ADF_BAND", "1"), ("ADF_BAND", "0") }, sent);
    }

    [Fact]
    public void FastDetentsStepFromTheBandAskedForNotTheOneReportedBack()
    {
        var (state, panel, radio, sent) = Create();
        TuneAdf(state, ADFMode.ADF);
        state.adfBand = ADFBand.Low;
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        panel.Press(RadioPanelKey.FineUp);                  // DCS has not answered yet
        panel.Press(RadioPanelKey.FineUp);

        Assert.Equal(new[] { ("ADF_BAND", "1"), ("ADF_BAND", "2") }, sent);
    }

    [Fact]
    public void BandKeysDoNothingOutsideTheAdf()
    {
        var (state, panel, radio, sent) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();

        panel.Press(RadioPanelKey.FineUp);                  // tunes the VHF standby instead

        Assert.DoesNotContain(sent, s => s.Item1 == "ADF_BAND");
    }

    [Fact]
    public void CommunicationKeyBringsTheRadioBack()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        panel.Press(RadioPanelKey.ShowComm);

        Assert.Equal("123.450", panel.Active);
    }

    [Fact]
    public void MovingTheSelectorTakesPriorityOverTheAdf()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        state.IntMode = Uhf;
        state.UhfFunction = UHFFunctionDial.TR;
        state.UhfMode = UHFModeDial.MANUAL;
        state.UhfFreq = "251.000";
        radio.Refresh();

        Assert.Equal("251.000", panel.Active);
    }

    [Fact]
    public void AdfStaysShownWhenOtherCockpitValuesChange()
    {
        var (state, panel, radio, _) = Create();
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();
        panel.Press(RadioPanelKey.ShowAdf);

        state.VhfFreq = "130.000";                          // a radio value moves, the selector does not
        radio.Refresh();

        Assert.Equal("443", panel.Active);
    }

    private static (bool? Vhf1, bool? Vhf2, bool? Vhf3, bool? Adf) Keys(FlightDeckState deck) =>
        (deck.GetSignal(FlightDeckSignal.RadioSlot1), deck.GetSignal(FlightDeckSignal.RadioSlot2),
         deck.GetSignal(FlightDeckSignal.RadioSlot3), deck.GetSignal(FlightDeckSignal.RadioAdf));

    [Fact]
    public void TheKeyOfTheSelectedRadioIsLit()
    {
        var deck = new FlightDeckState();
        var (state, _, radio, _) = Create(deck);
        Assert.Equal((true, false, false, false), Keys(deck));          // VHF AM -> VHF1

        state.IntMode = Uhf;
        radio.Refresh();
        Assert.Equal((false, true, false, false), Keys(deck));          // UHF -> VHF2

        state.IntMode = Fm;
        radio.Refresh();
        Assert.Equal((false, false, true, false), Keys(deck));          // FM -> VHF3
    }

    [Theory]
    [InlineData(Pvt)]
    [InlineData(Int)]
    [InlineData(Unused)]
    public void NoKeyIsLitWhenNoRadioIsSelected(int intMode)
    {
        var deck = new FlightDeckState();
        var (state, _, radio, _) = Create(deck);

        state.IntMode = intMode;
        radio.Refresh();

        Assert.Equal((false, false, false, false), Keys(deck));
    }

    [Fact]
    public void TheSelectedRadioKeyStaysLitWhenThatRadioIsOff()
    {
        var deck = new FlightDeckState();
        var (state, _, radio, _) = Create(deck);

        state.VhfPwr = false;
        radio.Refresh();

        Assert.Equal((true, false, false, false), Keys(deck));
    }

    [Fact]
    public void AdfKeyReplacesTheCommunicationKeyWhileTheAdfIsShown()
    {
        var deck = new FlightDeckState();
        var (state, panel, radio, _) = Create(deck);
        TuneAdf(state, ADFMode.ADF);
        radio.Refresh();

        panel.Press(RadioPanelKey.ShowAdf);
        Assert.Equal((false, false, false, true), Keys(deck));

        panel.Press(RadioPanelKey.ShowComm);
        Assert.Equal((true, false, false, false), Keys(deck));
    }

    [Fact]
    public void KeysDoNothingWhenNoRadioIsSelected()
    {
        var (state, panel, radio, sent) = Create();
        state.IntMode = Pvt;
        radio.Refresh();

        panel.Press(RadioPanelKey.CoarseUp);
        panel.Press(RadioPanelKey.Swap);

        state.IntMode = VhfAm;
        radio.Refresh();
        Assert.Equal("123.450", panel.Active);
        Assert.Equal("123.450", panel.Standby);
        Assert.Empty(sent);
    }
}
