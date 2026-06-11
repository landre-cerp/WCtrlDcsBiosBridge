using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

/// <summary>
/// Displays the aircraft selection menu on a CDU device. The entries are generated
/// from <see cref="AircraftRegistry"/> and laid out on the line-select keys,
/// left/right alternating, top to bottom.
/// </summary>
internal class AircraftSelectionMenu : IDisposable
{
    // Slot i maps to entry i: even = left column, odd = right column, row = i/2 + 1.
    private static readonly Key[] _slotKeys =
    {
        Key.LineSelectLeft1, Key.LineSelectRight1,
        Key.LineSelectLeft2, Key.LineSelectRight2,
        Key.LineSelectLeft3, Key.LineSelectRight3,
        Key.LineSelectLeft4, Key.LineSelectRight4,
        Key.LineSelectLeft5, Key.LineSelectRight5,
        Key.LineSelectLeft6, Key.LineSelectRight6,
    };

    private readonly ICdu mcdu;
    private readonly IReadOnlyList<AircraftMenuEntry> entries;
    private bool isActive;

    public event EventHandler<AircraftSelectedEventArgs>? AircraftSelected;

    public AircraftSelectionMenu(ICdu mcdu, bool ch47SwitchWithSeat = false)
    {
        this.mcdu = mcdu;

        var allEntries = AircraftRegistry.BuildMenuEntries(ch47SwitchWithSeat);
        if (allEntries.Count > _slotKeys.Length)
        {
            App.Logger.Warn($"Aircraft menu has {allEntries.Count} entries but only {_slotKeys.Length} line-select keys; extra entries are not shown.");
            allEntries = allEntries.Take(_slotKeys.Length).ToList();
        }
        entries = allEntries;
    }

    public void Show()
    {
        if (isActive) return;

        DisplayMenu();
        AttachEventHandlers();
        isActive = true;
    }

    public void Hide()
    {
        if (!isActive) return;

        DetachEventHandlers();
        isActive = false;
    }

    private void DisplayMenu()
    {
        var version = AppVersionProvider.GetAppVersion();

        var output = mcdu.Output.Clear().Green()
            .Line(0).Centered("DCSbios/WW Bridge")
            .NewLine().Large().Yellow().Centered("by Cerppo")
            .White();

        for (int i = 0; i < entries.Count; i++)
        {
            int row = i / 2 + 1;
            if (i % 2 == 0)
                output.LeftLabel(row, entries[i].Label);
            else
                output.RightLabel(row, entries[i].Label);
        }

        output.BottomLine().WriteLine($"v{version}");
        // Skip the duplicate check: after an unclean shutdown the device can be out
        // of sync with the library's cache, and the menu must always reach the panel.
        mcdu.RefreshDisplay(skipDuplicateCheck: true);
    }

    private void AttachEventHandlers() => mcdu.KeyDown += HandleKeyDown;
    private void DetachEventHandlers() => mcdu.KeyDown -= HandleKeyDown;

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
        var slot = Array.IndexOf(_slotKeys, e.Key);
        if (slot < 0 || slot >= entries.Count) return;

        Hide();
        AircraftSelected?.Invoke(this, new AircraftSelectedEventArgs(entries[slot].Selection));
    }

    public void Dispose() => Hide();
}

public sealed record AircraftSelection(int AircraftId, bool IsPilot);

public class AircraftSelectedEventArgs : EventArgs
{
    public AircraftSelection Selection { get; }

    public AircraftSelectedEventArgs(AircraftSelection selection)
    {
        Selection = selection;
    }
}
