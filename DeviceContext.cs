using WwDevicesDotNet;
using WWCduDcsBiosBridge.Aircrafts;
using WWCduDcsBiosBridge.Config;

namespace WWCduDcsBiosBridge;

/// <summary>
/// Represents the context for a CDU device within the bridge: it shows the aircraft
/// selection menu and runs the aircraft listener that drives the CDU display.
/// Frontpanel devices are not contexts — they are rendered by the FrontpanelHub
/// from one listener's FlightDeck model.
/// </summary>
internal class DeviceContext : IDisposable
{
    public ICdu Mcdu { get; }

    public AircraftSelection? SelectedAircraft { get; private set; }
    public bool IsSelectedAircraft { get => isSelectedAircraft; }

    /// <summary>
    /// The aircraft listener, available after <see cref="StartBridge"/> succeeded.
    /// </summary>
    public AircraftListener? Listener => listener;

    /// <summary>
    /// Completes when an aircraft has been selected on this CDU.
    /// </summary>
    public Task<AircraftSelection> SelectionTask => _selectionTcs.Task;

    private readonly TaskCompletionSource<AircraftSelection> _selectionTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private readonly UserOptions options;
    private readonly bool ch47SwitchWithSeat;
    private readonly AircraftSelectionMenu menu;
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

    public void ShowStartupScreen()
    {
        // The CDU keeps brightness/display state across app restarts. If a previous
        // session ended uncleanly (or Cleanup() zeroed the brightness), the menu
        // would render invisibly — force a known-visible state first.
        if (!options.DisableLightingManagement)
        {
            Mcdu.DisplayBrightnessPercent = 100;
            Mcdu.BacklightBrightnessPercent = 100;
            Mcdu.RefreshBrightnesses();
        }

        menu.Show();
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

    public void StartBridge()
    {
        if (!isSelectedAircraft || SelectedAircraft == null) return;

        try
        {
            listener = new AircraftListenerFactory().CreateListener(SelectedAircraft, Mcdu, options, ch47SwitchWithSeat);
            listener.Start();
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, $"Failed to start listener for aircraft {SelectedAircraft.AircraftId}");
            Mcdu.Output.Newline().Red().WriteLine(ex.Message);
            Mcdu.RefreshDisplay();
        }
    }

    public void Dispose()
    {
        menu.Dispose();
        listener?.Dispose();
    }
}
