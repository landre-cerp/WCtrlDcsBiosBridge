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

    public event EventHandler<AircraftSelection>? AircraftSelected;

    public AircraftSelectionPanel()
    {
        InitializeComponent();
        DataContext = this;
        SelectAircraftCommand = new RelayCommand<string>(OnAircraftSelected);
    }

    private void OnAircraftSelected(string? tag)
    {
        if (tag is null) return;

        var selection = tag switch
        {
            "A10C" => new AircraftSelection(SupportedAircrafts.A10C, true),
            "AH64D" => new AircraftSelection(SupportedAircrafts.AH64D, true),
            "FA18C" => new AircraftSelection(SupportedAircrafts.FA18C, true),
            "CH47_PLT" => new AircraftSelection(SupportedAircrafts.CH47, true),
            "CH47_CPLT" => new AircraftSelection(SupportedAircrafts.CH47, false),
            "F15E" => new AircraftSelection(SupportedAircrafts.F15E, true),
            "M2000C" => new AircraftSelection(SupportedAircrafts.M2000C, true),
            _ => null
        };

        var displayName = tag switch
        {
            "A10C" => "A-10C",
            "AH64D" => "AH-64D",
            "FA18C" => "F/A-18C",
            "CH47_PLT" => "CH-47F (PLT)",
            "CH47_CPLT" => "CH-47F (CPLT)",
            "F15E" => "F-15E",
            "M2000C" => "M-2000C",
            _ => tag
        };

        if (selection is not null)
        {
            SelectionStatus = $"Selected: {displayName}";
            SelectionStatusColor = Brushes.Green;
            ButtonsEnabled = false;
            AircraftSelected?.Invoke(this, selection);
        }
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
