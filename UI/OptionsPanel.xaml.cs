using System.Windows;
using System.Windows.Controls;
using WwDevicesDotNet;
namespace WWCduDcsBiosBridge.UI;

public partial class OptionsPanel : UserControl
{
    private bool _isInitializing = true;
    public event EventHandler? SettingsChanged;

    public OptionsPanel()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
        InitializeKeyComboBoxes();
    }

    private void InitializeKeyComboBoxes()
    {
        // Get all available MCDU keys as strings
        var keyNames = Enum.GetNames(typeof(Key));

        // Find the combo boxes by name and populate them
        if (FindName("NextPageKeyComboBox") is ComboBox nextPageCombo)
        {
            nextPageCombo.ItemsSource = keyNames;
        }

        if (FindName("PrevPageKeyComboBox") is ComboBox prevPageCombo)
        {
            prevPageCombo.ItemsSource = keyNames;
        }

        if (FindName("F16CNextDisplayKeyComboBox") is ComboBox f16cNextCombo)
        {
            f16cNextCombo.ItemsSource = keyNames;
        }

        if (FindName("F16CPrevDisplayKeyComboBox") is ComboBox f16cPrevCombo)
        {
            f16cPrevCombo.ItemsSource = keyNames;
        }

        if (FindName("F16CRwrDisplayKeyComboBox") is ComboBox f16cRwrCombo)
        {
            f16cRwrCombo.ItemsSource = keyNames;
        }
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        _isInitializing = true;
        Dispatcher.BeginInvoke(new Action(() => _isInitializing = false), System.Windows.Threading.DispatcherPriority.DataBind);
    }

    private void CheckBox_Changed(object sender, RoutedEventArgs e)
    {
        // Only notify parent if this is a user-initiated change (not during initialization)
        if (!_isInitializing)
        {
            SettingsChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    private void ComboBox_Changed(object sender, SelectionChangedEventArgs e)
    {
        if (!_isInitializing)
        {
            SettingsChanged?.Invoke(this, EventArgs.Empty);
        }
    }

}
