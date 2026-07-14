using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Devices.Cdu;

/// <summary>
/// Represents the context for a CDU device within the bridge: it shows the aircraft
/// selection menu and runs the aircraft listener that drives the CDU display.
/// Frontpanel devices are not contexts — they are rendered by the FrontpanelHub
/// from one listener's FlightDeck model.
/// </summary>
internal class CduDeviceContext : IDisposable
{
    public ICdu Mcdu { get; }

    public AircraftSelection? SelectedAircraft { get; private set; }
    public bool IsSelectedAircraft => isSelectedAircraft;

    /// <summary>
    /// The aircraft listener, available after <see cref="StartBridge"/> succeeded.
    /// </summary>
    public AircraftListener? Listener => listener;

    /// <summary>
    /// Completes when an aircraft has been selected on this CDU.
    /// </summary>
    public Task<AircraftSelection> SelectionTask => _selectionTcs.Task;

    private TaskCompletionSource<AircraftSelection> _selectionTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private readonly UserOptions options;
    private readonly bool ch47SwitchWithSeat;
    private readonly AircraftCduContext cduContext;
    private AircraftListener? listener;
    private bool isSelectedAircraft = false;
    private EventHandler<KeyEventArgs>? _seatKeyHandler;

    /// <summary>
    /// Creates a context for a CDU device.
    /// </summary>
    /// <param name="ch47SwitchWithSeat">
    /// True when a single CDU is connected: the CH-47F display follows the
    /// seat position at runtime.
    /// </param>
    public CduDeviceContext(ICdu mcdu, UserOptions options, bool ch47SwitchWithSeat)
    {
        Mcdu = mcdu;
        this.options = options;
        this.ch47SwitchWithSeat = ch47SwitchWithSeat;
        cduContext = new AircraftCduContext(mcdu);
    }

    /// <summary>
    /// Resets the CDU (clears managed lighting state) unless the user disabled
    /// lighting management. Shared by the waiting and seat-selection screens.
    /// </summary>
    private void ResetIfLightingManaged()
    {
        if (!options.DisableLightingManagement)
        {
            cduContext.Reset();
        }
    }

    /// <summary>
    /// Displays the "Waiting for DCS..." screen while the bridge watches DCS-BIOS
    /// metadata for a supported aircraft. The aircraft line is blank until a module
    /// is detected; an unsupported module shows its raw name and an error in red
    /// (the bridge keeps waiting for a supported one). The app version stays white
    /// on the bottom line, with the live DCS-BIOS exporter version just above it.
    /// </summary>
    public void ShowWaitingScreen(string? unsupportedName = null, string? dcsBiosVersion = null)
    {
        ResetIfLightingManaged();

        var version = AppVersionProvider.GetAppVersion();
        var dcsBios = string.IsNullOrEmpty(dcsBiosVersion) ? "--.--" : dcsBiosVersion;
        var output = Mcdu.Output.Clear().Green()
            .Line(0).Centered("DCSbios/WW Bridge")
            .NewLine().Large().Yellow().Centered("by Cerppo")
            .NewLine().NewLine().White()
            .Centered("Waiting for DCS...")
            .NewLine()
            .Centered("Aircraft detection");

        // Reserved aircraft line (blank until a module is detected).
        output.NewLine().NewLine();
        if (!string.IsNullOrEmpty(unsupportedName))
        {
            output.Red()
                .Centered(unsupportedName)
                .NewLine()
                .Centered("Not supported");
        }

        // DCS-BIOS exporter version on the line just above the app version.
        output.White().Line(-1).Centered($"DCS-BIOS {dcsBios}");
        output.White().BottomLine().WriteLine($"v{version}");
        Mcdu.RefreshDisplay(skipDuplicateCheck: true);
    }

    /// <summary>
    /// Shows a pilot / copilot seat selection screen for the given aircraft.
    /// Used when a dual-seat aircraft (e.g. CH-47F) is detected and multiple
    /// CDUs are connected — each CDU picks a seat.
    /// </summary>
    public void ShowSeatSelectionScreen(AircraftDescriptor descriptor)
    {
        ResetIfLightingManaged();

        var version = AppVersionProvider.GetAppVersion();
        Mcdu.Output.Clear().Green()
            .Line(0).Centered("DCSbios/WW Bridge")
            .NewLine().Large().Yellow().Centered(descriptor.DisplayName)
            .NewLine().NewLine().White()
            .Centered("Select your seat:")
            .LeftLabel(3, "PILOT")
            .RightLabel(3, "COPILOT")
            .BottomLine().WriteLine($"v{version}");
        Mcdu.RefreshDisplay(skipDuplicateCheck: true);

        // Replace any previous handler (e.g. a prior selection that was abandoned).
        DetachSeatKeyHandler();
        _seatKeyHandler = SeatKeyHandler;
        Mcdu.KeyDown += _seatKeyHandler;

        void SeatKeyHandler(object? sender, KeyEventArgs e)
        {
            bool? isPilot = e.Key switch
            {
                Key.LineSelectLeft3 => true,
                Key.LineSelectRight3 => false,
                _ => null
            };

            if (isPilot == null) return;

            DetachSeatKeyHandler();
            SetAircraftSelection(new AircraftSelection(descriptor.ModuleId, isPilot.Value));
        }
    }

    /// <summary>
    /// Detaches the seat-selection key handler if one is attached, so an
    /// abandoned selection (e.g. the aircraft was exited before a seat was
    /// chosen) leaves no stale subscription on the device.
    /// </summary>
    private void DetachSeatKeyHandler()
    {
        if (_seatKeyHandler != null)
        {
            Mcdu.KeyDown -= _seatKeyHandler;
            _seatKeyHandler = null;
        }
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

    public void StartBridge()
    {
        if (!isSelectedAircraft || SelectedAircraft == null) return;

        try
        {
            listener = new AircraftListenerFactory().CreateListener(SelectedAircraft, cduContext, options, ch47SwitchWithSeat);
            listener.Start();
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, $"Failed to start listener for aircraft {SelectedAircraft.AircraftId}");
            Mcdu.Output.Newline().Red().WriteLine(ex.Message);
            Mcdu.RefreshDisplay();
        }
    }

    /// <summary>
    /// Stops the current listener and clears selection state so this context
    /// can be re-used for a new aircraft detection cycle.
    /// </summary>
    public void ResetForNewCycle()
    {
        DetachSeatKeyHandler();
        listener?.Dispose();
        listener = null;
        isSelectedAircraft = false;
        SelectedAircraft = null;
        _selectionTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
    }

    public void Dispose()
    {
        DetachSeatKeyHandler();
        listener?.Dispose();
    }
}
