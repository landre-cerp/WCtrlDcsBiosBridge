using System.Windows;
using System.Windows.Controls;
using WwDevicesDotNet;
using WWCduDcsBiosBridge.Config;

namespace WWCduDcsBiosBridge.UI;

public partial class OptionsPanel : UserControl
{
    private bool _isInitializing = true;
    public event EventHandler? SettingsChanged;
    public event EventHandler<ThemePreference>? ThemePreferenceChanged;

    public OptionsPanel()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
        InitializeKeyComboBoxes();
    }

    private void InitializeKeyComboBoxes()
    {
        if (FindName("ThemeComboBox") is ComboBox themeCombo)
        {
            themeCombo.ItemsSource = new[] { "System (follow Windows)", "Light", "Dark" };
        }

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

        if (DataContext is UserOptions opts && FindName("ThemeComboBox") is ComboBox themeCombo)
        {
            themeCombo.SelectedIndex = (int)opts.Theme;
        }

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

    private void ThemeComboBox_Changed(object sender, SelectionChangedEventArgs e)
    {
        if (_isInitializing) return;
        if (DataContext is not UserOptions options) return;
        if (sender is not ComboBox combo) return;
        options.Theme = (ThemePreference)combo.SelectedIndex;
        ThemePreferenceChanged?.Invoke(this, options.Theme);
        SettingsChanged?.Invoke(this, EventArgs.Empty);
    }
}
