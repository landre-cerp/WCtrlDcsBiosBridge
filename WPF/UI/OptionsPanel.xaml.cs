using System.Windows;
using System.Windows.Controls;
using WwDevicesDotNet;
namespace WCtrlDcsBiosBridge.UI;

public partial class OptionsPanel : UserControl
{
    private bool _isInitializing = true;
    public event EventHandler? SettingsChanged;

    /// <summary>
    /// All MCDU key names, for binding the key dropdowns straight from XAML
    /// (<c>ItemsSource="{x:Static ui:OptionsPanel.McduKeyNames}"</c>) — so a new
    /// dropdown never needs code-behind.
    /// </summary>
    public static IReadOnlyList<string> McduKeyNames { get; } = Enum.GetNames(typeof(Key));

    public OptionsPanel()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
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
