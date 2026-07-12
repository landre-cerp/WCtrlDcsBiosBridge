using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using WCtrlDcsBiosBridge.Common;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.UI;

public partial class OptionsPanel : UserControl
{
    private bool _isInitializing = true;
    public event EventHandler? SettingsChanged;

    /// <summary>All MCDU key names, used to populate the key dropdowns.</summary>
    public static IReadOnlyList<string> McduKeyNames { get; } = Enum.GetNames(typeof(Key));

    private readonly Dictionary<string, (ContentControl Section, Border Badge)> _aircraftSections;

    public OptionsPanel()
    {
        InitializeComponent();

        foreach (var combo in new[]
                 {
                     A10CPerfPageKeyCombo, A10CNextPageKeyCombo, A10CPrevPageKeyCombo,
                     FA18CShowIfeiKeyCombo, FA18CShowUfcKeyCombo,
                     F14RioKeyCombo, F14RadioKeyCombo,
                     F16CDedKeyCombo, F16CNavKeyCombo, F16CRwrKeyCombo
                 })
        {
            combo.ItemsSource = McduKeyNames;
        }

        _aircraftSections = new Dictionary<string, (ContentControl, Border)>(StringComparer.Ordinal)
        {
            ["A-10C"] = (A10CSection, A10CBadge),
            ["F/A-18C"] = (FA18CSection, FA18CBadge),
            ["F-14B"] = (F14Section, F14Badge),
            ["F-16C"] = (F16CSection, F16CBadge),
        };

        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(FrameworkElement sender, DataContextChangedEventArgs args)
    {
        _isInitializing = true;
        DispatcherQueue.TryEnqueue(() =>
        {
            _isInitializing = false;
            A10CPerfPagesPanel.IsEnabled = A10CEnablePerfPagesCheckBox.IsChecked == true;
        });
    }

    /// <summary>Greys out (with a badge) the section for the aircraft currently running the bridge.</summary>
    public void SetDetectedAircraft(string? detectedAircraftName)
    {
        foreach (var (name, (section, badge)) in _aircraftSections)
        {
            bool isActive = string.Equals(name, detectedAircraftName, StringComparison.Ordinal);
            section.IsEnabled = !isActive;
            badge.Visibility = isActive ? Visibility.Visible : Visibility.Collapsed;
        }
    }

    /// <summary>Greys out the "on close" reset section when lighting management is disabled.</summary>
    public void SetLightingManaged(bool isLightingManaged)
    {
        OnCloseSection.IsEnabled = isLightingManaged;
        OnCloseBadge.Visibility = isLightingManaged ? Visibility.Collapsed : Visibility.Visible;
    }

    private void CheckBox_Changed(object sender, RoutedEventArgs e)
    {
        if (!_isInitializing)
        {
            SettingsChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    private void A10CEnablePerfPages_Changed(object sender, RoutedEventArgs e)
    {
        A10CPerfPagesPanel.IsEnabled = A10CEnablePerfPagesCheckBox.IsChecked == true;
        CheckBox_Changed(sender, e);
    }

    private void ComboBox_Changed(object sender, SelectionChangedEventArgs e)
    {
        if (!_isInitializing)
        {
            SettingsChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    /// <summary>Re-applies every static string from Strings\&lt;lang&gt;\Resources.resw.</summary>
    public void Retranslate()
    {
        GeneralHeader.Text = Strings.Get("GeneralHeader");
        DisableLightingManagementOption.Content = Strings.Get("DisableLightingManagementOption");
        AutoStartOption.Content = Strings.Get("AutoStartOption");
        MinimizeOnStartOption.Content = Strings.Get("MinimizeOnStartOption");

        OnCloseHeader.Text = Strings.Get("OnCloseHeader");
        LightingManagementDisabledBadge.Text = Strings.Get("LightingManagementDisabledBadge");
        TurnOffBacklightsOption.Content = Strings.Get("TurnOffBacklightsOption");
        TurnOffMarkersOption.Content = Strings.Get("TurnOffMarkersOption");

        DisabledWhileInUseA10CBadge.Text = Strings.Get("DisabledWhileInUseA10CBadge");
        AlignCduBottomOption.Content = Strings.Get("AlignCduBottomOption");
        DisplayCmsOption.Content = Strings.Get("DisplayCmsOption");
        A10CEnablePerfPagesCheckBox.Content = Strings.Get("EnablePerfPagesOption");
        PerfPageKeyLabel.Text = Strings.Get("PerfPageKeyLabel");
        NextPageKeyLabel.Text = Strings.Get("NextPageKeyLabel");
        PrevPageKeyLabel.Text = Strings.Get("PrevPageKeyLabel");
        ForwardCduKeysOption.Content = Strings.Get("ForwardCduKeysOption");

        DisabledWhileInUseFA18CBadge.Text = Strings.Get("DisabledWhileInUseFA18CBadge");
        IfeiPageNavHeader.Text = Strings.Get("IfeiPageNavHeader");
        ShowIfeiKeyLabel.Text = Strings.Get("ShowIfeiKeyLabel");
        ShowUfcKeyLabel.Text = Strings.Get("ShowUfcKeyLabel");

        DisabledWhileInUseF14Badge.Text = Strings.Get("DisabledWhileInUseF14Badge");
        RioCapRadioHeader.Text = Strings.Get("RioCapRadioHeader");
        ShowRioKeyLabel.Text = Strings.Get("ShowRioKeyLabel");
        ShowRadioKeyLabel.Text = Strings.Get("ShowRadioKeyLabel");

        DisabledWhileInUseF16CBadge.Text = Strings.Get("DisabledWhileInUseF16CBadge");
        DedNavRwrHeader.Text = Strings.Get("DedNavRwrHeader");
        ShowDedKeyLabel.Text = Strings.Get("ShowDedKeyLabel");
        ShowNavKeyLabel.Text = Strings.Get("ShowNavKeyLabel");
        ShowRwrKeyLabel.Text = Strings.Get("ShowRwrKeyLabel");
    }
}
