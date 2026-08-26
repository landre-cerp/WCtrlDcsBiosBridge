namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Follows the CNI-MU's EXEC annunciator, which nothing exports.
///
/// It is not an animation argument — a sweep of arguments 0..3000 across a full EXEC cycle
/// shows only the key's own press pulse and nothing that stays — and it is not in
/// <c>list_cockpit_params</c> either: the same 155 keys before and after the module's
/// 2026-08-26 update, none of them CNI. So it is worked out from two things that are
/// exported, and it has to be remembered rather than recomputed from the page in hand.
///
/// The first is the page title. The module builds the titles of its nineteen modifiable pages
/// from a format with a leading <c>%s</c> — <c>"%sRTE %d"</c> in the C-130J's own route_gen.lua
/// — and the sim fills that in with "MOD " while a change waits and "ACT " once it has gone
/// through. One observed cycle: "RTE 1", then "MOD RTE 1" as the route change is entered and
/// the lamp lights, then "ACT RTE 1" on the EXEC press as it goes out.
///
/// That alone would put the lamp out every time the crew turned to another page, because the
/// marker is only ever on the modified one. Hence the latch: only evidence flips it, and a
/// page with nothing to say about the modification leaves it as it was.
///
/// The second is the EXEC key. The lamp is not readable but the key is — any cockpit argument
/// is, whether or not anything declares it — so the export counts presses on all three CNIs and
/// this takes each one as the modification having gone through. That is what covers executing
/// from a page that is not the modified one, where the title being watched never moves.
///
/// Discarding the change instead is covered by the title alone: ERASE is only ever on the
/// modified page, so its marker disappears while that page is the one on screen.
/// </summary>
internal sealed class CniExecLamp
{
    /// <summary>
    /// What the sim puts in front of a title while that page holds an unexecuted change. The
    /// trailing space is part of it: it separates the marker from the title the format builds,
    /// and requiring it keeps a page whose own title merely starts with those letters — a MODE
    /// page — from lighting the lamp.
    /// </summary>
    private const string ModifiedPrefix = "MOD ";

    /// <summary>The other marker the same slot takes, once the change has been executed.</summary>
    private const string ActivePrefix = "ACT ";

    /// <summary>Whether the annunciator is lit.</summary>
    public bool Armed { get; private set; }

    /// <summary>
    /// The page the marker was last seen on, stripped of that marker. What makes an unmarked
    /// title mean something: on the page that armed the lamp it says the change is gone, and on
    /// any other page it says nothing at all.
    /// </summary>
    private string? _markedPage;

    /// <summary>
    /// The press count last seen. Held rather than compared against zero because the count
    /// belongs to the export script's run, not to this listener's — the app can attach to a
    /// mission already under way.
    /// </summary>
    private int? _presses;

    /// <summary>
    /// Takes one packet and returns the lamp's state. <paramref name="execPresses"/> is absent
    /// on a script too old to send it, which costs the key evidence and nothing else.
    /// </summary>
    public bool Update(string? title, int? execPresses)
    {
        // Pressed first, page second: on the packet where both land they agree — the key went
        // down and the title turned from MOD to ACT — and elsewhere the order does not arise.
        if (execPresses is { } count)
        {
            // Any difference, not an increase. A mission restart takes the script's counter
            // back to zero, and reading that as "no press" would leave a lamp latched on with
            // nothing left in the session able to put it out.
            if (_presses is { } previous && previous != count) Release();
            _presses = count;
        }

        var page = PageOf(title);

        if (IsMarked(title))
        {
            Armed = true;
            _markedPage = page;
        }
        else if (Armed && _markedPage is not null && string.Equals(_markedPage, page, StringComparison.Ordinal))
        {
            Release();
        }

        return Armed;
    }

    private void Release()
    {
        Armed = false;
        _markedPage = null;
    }

    private static bool IsMarked(string? title) =>
        title is not null && title.TrimStart().StartsWith(ModifiedPrefix, StringComparison.Ordinal);

    /// <summary>
    /// The title with its marker taken off, so that the three forms of one page — bare, MOD and
    /// ACT — are recognisably the same page. Blanks are trimmed on both sides of the marker:
    /// CUSTOM DATA builds its title as <c>"%s CUSTOM DATA"</c>, so the unmarked page opens with
    /// a blank that the marked one carries in the middle.
    /// </summary>
    private static string? PageOf(string? title)
    {
        if (title is null) return null;

        var text = title.TrimStart();

        foreach (var marker in new[] { ModifiedPrefix, ActivePrefix })
        {
            if (text.StartsWith(marker, StringComparison.Ordinal))
                return text[marker.Length..].TrimStart();
        }

        return text;
    }
}
