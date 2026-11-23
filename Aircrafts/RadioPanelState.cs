namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// What a radio panel can ask of the aircraft, whatever the panel looks like. A renderer maps
/// its device's buttons and rings onto these.
/// </summary>
internal enum RadioPanelKey
{
    /// <summary>Exchange the active and standby displays.</summary>
    Swap,

    /// <summary>Standby frequency up by one MHz.</summary>
    CoarseUp,

    /// <summary>Standby frequency down by one MHz.</summary>
    CoarseDown,

    /// <summary>Standby frequency up by one channel step.</summary>
    FineUp,

    /// <summary>Standby frequency down by one channel step.</summary>
    FineDown,

    /// <summary>Copy the active frequency into the standby.</summary>
    Sync,

    /// <summary>Show the ADF (read only) until a communication key is pressed.</summary>
    ShowAdf,

    /// <summary>Back to the communication radio the aircraft's own selector points at.</summary>
    ShowComm,
}

/// <summary>
/// The two frequencies a radio panel shows — active and standby — and the way its keys reach
/// the aircraft. An aircraft listener writes the text and installs the key handler; the panel's
/// renderer reads the text and forwards the keys. Both run on different threads.
/// </summary>
internal sealed class RadioPanelState
{
    private readonly object _gate = new();
    private string _active = string.Empty;
    private string _standby = string.Empty;

    /// <summary>Receives the panel's keys. Set once by the listener when it starts.</summary>
    public Action<RadioPanelKey>? KeyHandler { get; set; }

    public string Active
    {
        get { lock (_gate) return _active; }
    }

    public string Standby
    {
        get { lock (_gate) return _standby; }
    }

    /// <summary>Sets both displays. Empty blanks one.</summary>
    public void Show(string? active, string? standby)
    {
        lock (_gate)
        {
            _active = active ?? string.Empty;
            _standby = standby ?? string.Empty;
        }
    }

    /// <summary>Forwards a key from the panel to the aircraft.</summary>
    public void Press(RadioPanelKey key) => KeyHandler?.Invoke(key);
}
