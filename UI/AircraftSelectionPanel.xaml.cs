using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using WWCduDcsBiosBridge.Aircrafts;

namespace WWCduDcsBiosBridge.UI;

public partial class AircraftSelectionPanel : UserControl, INotifyPropertyChanged
{
    private string _headerMessage = "No CDU connected. Select an aircraft to start the bridge:";
    private string _selectionStatus = "No aircraft selected";
    private Brush _selectionStatusColor = Brushes.Orange;
    private bool _buttonsEnabled = true;

    public string HeaderMessage
    {
        get => _headerMessage;
        set { _headerMessage = value; OnPropertyChanged(); }
    }

    public string SelectionStatus
    {
        get => _selectionStatus;
        set { _selectionStatus = value; OnPropertyChanged(); }
    }

    public Brush SelectionStatusColor
    {
        get => _selectionStatusColor;
        set { _selectionStatusColor = value; OnPropertyChanged(); }
    }

    public bool ButtonsEnabled
    {
        get => _buttonsEnabled;
        set { _buttonsEnabled = value; OnPropertyChanged(); }
    }

    public ICommand SelectAircraftCommand { get; }

    /// <summary>
    /// Selectable aircraft generated from the registry. The panel is only shown
    /// when no CDU is connected, so seat-selection aircraft always get a PLT/CPLT pair.
    /// </summary>
    public IReadOnlyList<AircraftMenuEntry> AircraftEntries { get; } =
        AircraftRegistry.BuildMenuEntries(ch47SwitchWithSeat: false);

    public event EventHandler<AircraftSelection>? AircraftSelected;

    public AircraftSelectionPanel()
    {
        InitializeComponent();
        DataContext = this;
        SelectAircraftCommand = new RelayCommand<AircraftMenuEntry>(OnAircraftSelected);
    }

    private void OnAircraftSelected(AircraftMenuEntry? entry)
    {
        if (entry is null) return;

        SelectionStatus = $"Selected: {entry.Label}";
        SelectionStatusColor = Brushes.Green;
        ButtonsEnabled = false;
        AircraftSelected?.Invoke(this, entry.Selection);
    }

    public void Reset()
    {
        SelectionStatus = "No aircraft selected";
        SelectionStatusColor = Brushes.Orange;
        ButtonsEnabled = true;
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    protected virtual void OnPropertyChanged([CallerMemberName] string? propertyName = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
