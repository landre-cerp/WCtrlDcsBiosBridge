using WwDevicesDotNet;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Config;
using WWCduDcsBiosBridge.Frontpanels;


namespace WWCduDcsBiosBridge;

/// <summary>
/// Represents the context for a device (CDU or Frontpanel) within the bridge.
/// CDU devices show an aircraft selection menu, while Frontpanel devices automatically
/// participate once an aircraft is selected globally.
/// </summary>
internal class DeviceContext : IDisposable
{
    public ICdu? Mcdu { get; }
    public IFrontpanel? Frontpanel { get; }
    public bool IsCduDevice => Mcdu != null;
    public bool IsFrontpanelDevice => Frontpanel != null;

    public AircraftSelection? SelectedAircraft { get; private set; }
    public bool IsSelectedAircraft { get => isSelectedAircraft; }
    public bool Pilot { get; private set; } = true;

    /// <summary>
    /// Completes when an aircraft has been selected for this context.
    /// </summary>
    public Task<AircraftSelection> SelectionTask => _selectionTcs.Task;

    private readonly TaskCompletionSource<AircraftSelection> _selectionTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private readonly UserOptions options;
    private readonly bool ch47SwitchWithSeat;
    private readonly AircraftSelectionMenu? menu;
    private AircraftListener? listener;
    private bool isSelectedAircraft = false;

    /// <summary>
    /// Creates a context for a CDU device with aircraft selection menu
    /// </summary>
    /// <param name="ch47SwitchWithSeat">
    /// True when a single CDU is connected: the CH-47F menu shows one entry and the
    /// CDU display follows the seat position at runtime.
    /// </param>
    public DeviceContext(ICdu mcdu, UserOptions options, bool ch47SwitchWithSeat)
    {
        Mcdu = mcdu;
        this.options = options;
        this.ch47SwitchWithSeat = ch47SwitchWithSeat;
        menu = new AircraftSelectionMenu(mcdu, ch47SwitchWithSeat);
        menu.AircraftSelected += OnAircraftSelected;
    }

    /// <summary>
    /// Creates a context for a Frontpanel device without aircraft selection menu
    /// </summary>
    public DeviceContext(IFrontpanel frontpanel, UserOptions options)
    {
        Frontpanel = frontpanel;
        this.options = options;
        // Frontpanel devices don't show aircraft selection menu
        // They wait for global aircraft selection to be propagated
    }

    public void ShowStartupScreen()
    {
        if (IsCduDevice)
        {
            menu?.Show();
        }
        // Frontpanel devices don't show a startup screen
    }

    /// <summary>
    /// Sets the aircraft selection for this device context
    /// </summary>
    public void SetAircraftSelection(AircraftSelection selection)
    {
        isSelectedAircraft = true;
        SelectedAircraft = selection;
        _selectionTcs.TrySetResult(selection);
    }

    private void OnAircraftSelected(object? sender, AircraftSelectedEventArgs e)
    {
        SetAircraftSelection(e.Selection);
    }

    public void StartBridge(FrontpanelHub frontpanelHub)
    {
        if (!isSelectedAircraft || SelectedAircraft == null) return;

        try
        {
            listener = new AircraftListenerFactory().CreateListener(SelectedAircraft, Mcdu, options, frontpanelHub, ch47SwitchWithSeat);
            listener.Start();
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, $"Failed to start listener for aircraft {SelectedAircraft.AircraftId}");
            if (Mcdu != null)
            {
                Mcdu.Output.Newline().Red().WriteLine(ex.Message);
                Mcdu.RefreshDisplay();
            }
        }
    }

    public void Dispose()
    {
        menu?.Dispose();
        listener?.Dispose();
    }
}
