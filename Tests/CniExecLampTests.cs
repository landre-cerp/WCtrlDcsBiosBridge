using WCtrlDcsBiosBridge.Aircrafts.C130J;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// The CNI-MU's EXEC annunciator, which the sim reports nowhere and which is therefore held
/// rather than recomputed: the marker that lights it is only ever on the modified page, so a
/// lamp read off whatever page is on screen would go out the moment the crew turned away.
///
/// The titles here are the ones a live session produced either side of an EXEC press.
/// </summary>
public class CniExecLampTests
{
    /// <summary>No press count, as a script too old to send one leaves it.</summary>
    private static readonly int? NoKey = null;

    [Theory]
    [InlineData("MOD RTE 1")]
    [InlineData("MOD LEGS 1")]
    [InlineData("MOD RTE 1 HOLD")]
    [InlineData("MOD WAYPOINT DATA")]
    public void AModifiedPageLightsIt(string title) =>
        Assert.True(new CniExecLamp().Update(title, NoKey));

    [Theory]
    [InlineData("RTE 1")]
    [InlineData("ACT RTE 1")]
    [InlineData("COMM TUNE INDEX")]
    public void AnUnmodifiedPageLeavesItDark(string title) =>
        Assert.False(new CniExecLamp().Update(title, NoKey));

    [Fact]
    public void ExecutingOnThePageItselfPutsItOut()
    {
        var lamp = new CniExecLamp();

        Assert.True(lamp.Update("MOD RTE 1", 0));

        // The marker becomes ACT on the same page: the change went through.
        Assert.False(lamp.Update("ACT RTE 1", 1));
    }

    [Fact]
    public void ErasingThePageChangePutsItOut()
    {
        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 0);

        // ERASE is only ever offered on the modified page, so the marker goes while that page
        // is still the one on screen — and no key is pressed.
        Assert.False(lamp.Update("RTE 1", 0));
    }

    /// <summary>
    /// The case the latch exists for: the crew turns to another page with the change still
    /// pending. The lamp is lit in the cockpit and no page but the modified one says so.
    /// </summary>
    [Fact]
    public void TurningToAnotherPageHoldsIt()
    {
        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 0);

        Assert.True(lamp.Update("COMM TUNE INDEX", 0));
        Assert.True(lamp.Update("LEGS 1", 0));

        // Including a page of the same family that carries a marker slot but no marker: it is
        // the modification's own page that speaks for it, not its neighbours.
        Assert.True(lamp.Update("ACT RTE 2", 0));
    }

    /// <summary>
    /// And the question that latch raises: pressing EXEC from a page that is not the modified
    /// one. Nothing on screen moves, so the keypress is the only evidence there is.
    /// </summary>
    [Fact]
    public void ExecutingFromAnotherPagePutsItOut()
    {
        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 4);
        Assert.True(lamp.Update("COMM TUNE INDEX", 4));

        Assert.False(lamp.Update("COMM TUNE INDEX", 5));

        // And it stays out: the count holds at its new value.
        Assert.False(lamp.Update("COMM TUNE INDEX", 5));
    }

    /// <summary>
    /// A mission restart takes the script's counter back to zero. Read as an increase only,
    /// that press would be missed and the lamp would stay lit with nothing left to clear it.
    /// </summary>
    [Fact]
    public void ACounterGoingBackwardsStillCountsAsAPress()
    {
        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 9);

        Assert.False(lamp.Update("COMM TUNE INDEX", 0));
    }

    /// <summary>
    /// The first count seen says nothing — the app can attach to a mission already under way,
    /// where the crew has pressed EXEC a dozen times before the CDU was plugged in.
    /// </summary>
    [Fact]
    public void TheFirstCountIsNotAPress()
    {
        var lamp = new CniExecLamp();

        Assert.True(lamp.Update("MOD RTE 1", 12));
    }

    /// <summary>
    /// A page holding the marker keeps it lit on every heartbeat, and re-arms after a press if
    /// the crew makes a second change.
    /// </summary>
    [Fact]
    public void ASecondChangeLightsItAgain()
    {
        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 0);
        lamp.Update("ACT RTE 1", 1);

        Assert.True(lamp.Update("MOD RTE 1", 1));
        Assert.True(lamp.Update("MOD RTE 1", 1));
    }

    /// <summary>
    /// CUSTOM DATA builds its title as <c>"%s CUSTOM DATA"</c>, so the marker lands in front of
    /// a leading blank and the unmarked page starts with one. Both forms have to read as the
    /// same page, or the unmarked one would never put the lamp out.
    /// </summary>
    [Fact]
    public void ALeadingBlankInTheTitleChangesNothing()
    {
        var lamp = new CniExecLamp();

        Assert.True(lamp.Update("MOD  CUSTOM DATA", NoKey));
        Assert.False(lamp.Update(" CUSTOM DATA", NoKey));
    }

    /// <summary>
    /// The marker is "MOD " with its separator, not the three letters: a title of its own that
    /// begins with them is a page, not a pending change.
    /// </summary>
    [Fact]
    public void ATitleMerelyStartingWithThoseLettersIsNotAMarker()
    {
        Assert.False(new CniExecLamp().Update("MODE TEST", NoKey));
        Assert.False(new CniExecLamp().Update("MOD", NoKey));
    }

    /// <summary>
    /// One page draws no title at all, and a packet can reach the listener before the export
    /// has found one. Neither is a modifiable page, and neither says anything about one.
    /// </summary>
    [Fact]
    public void NoTitleSaysNothingEitherWay()
    {
        Assert.False(new CniExecLamp().Update(null, NoKey));
        Assert.False(new CniExecLamp().Update("", NoKey));

        var lamp = new CniExecLamp();
        lamp.Update("MOD RTE 1", 0);
        Assert.True(lamp.Update(null, 0));
    }
}
