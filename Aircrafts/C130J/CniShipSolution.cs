namespace WCtrlDcsBiosBridge.Aircrafts.C130J;

/// <summary>
/// Decides the corner digit — the one thing on a CNI page that is settled the same way on all
/// 145 of them.
///
/// Every page prints, top left, the number of the INAV feeding that crew station: 1 on the
/// pilot's, 2 on the copilot's. It is boxed when that INAV is the one the aircraft has settled
/// on, which is why the two are never boxed at once. Both forms are the same character in the
/// same place, so the CNI's own indication cannot tell them apart — switching SHIP SOLN on the
/// NAV SELECT page swaps the element behind the digit and changes nothing else on the page.
///
/// The comparison is possible because the two halves come from different places: the digit is
/// the page's own text, and the active solution is read off the PFD by the export.
/// </summary>
internal static class CniShipSolution
{
    /// <summary>The controller the module gives that corner, on every page.</summary>
    private const string Field = "cni_sp";

    /// <summary>
    /// The variant to draw, or null where this is not the corner digit or nothing is known
    /// about the solution — in which case the caller falls back to claiming nothing.
    /// </summary>
    public static CniSlot? Choose(CniSlot slot, CniSlot other, string? text, int? solution)
    {
        if (solution is not { } active) return null;
        if (slot.Source != Field && other.Source != Field) return null;

        var digit = text?.Trim();
        if (digit is not { Length: 1 } || !char.IsAsciiDigit(digit[0])) return null;

        var boxed = digit[0] - '0' == active;

        return slot.IsInvert == boxed ? slot : other;
    }
}
