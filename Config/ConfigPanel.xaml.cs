using System.Net;
using Microsoft.UI;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using WCtrlDcsBiosBridge.Common;
using WCtrlDcsBiosBridge.Services;
using Windows.Storage.Pickers;
using WinRT.Interop;

namespace WCtrlDcsBiosBridge.Config;

/// <summary>
/// In-window overlay panel (not a secondary Window) for editing DCS-BIOS config. Every
/// previous approach — ContentDialog, a raw IFileOpenDialog COM call, even the async
/// Windows.Storage.Pickers.FolderPicker — crashed intermittently (STATUS_FATAL_APP_EXIT
/// in Microsoft.UI.Xaml.dll) specifically because this was a *secondary* Window. Hosting
/// the same UI as a UserControl inside MainWindow removes the secondary-window/cross-window
/// continuation entirely, which was the actual common factor in every crash.
///
/// BrowseButton_Click uses the standard Windows.Storage.Pickers.FolderPicker rather than
/// the raw IFileOpenDialog COM workaround: the crash above is gone now that this is an
/// overlay, and the tradeoff (the WinRT picker's "Select Folder" button can stay disabled
/// until a row is re-clicked, even inside the target folder) is accepted as preferable to
/// the COM interop.
/// </summary>
public partial class ConfigPanel : UserControl
{
    private const int MinPortNumber = 1;
    private const int MaxPortNumber = 65535;

    /// <summary>Owning window's HWND, used only for the folder picker. Set once by MainWindow.</summary>
    public IntPtr OwnerHwnd { get; set; }

    public DcsBiosConfig Config { get; private set; } = new();

    private TaskCompletionSource<bool?>? _tcs;

    public ConfigPanel()
    {
        InitializeComponent();
        Retranslate();
    }

    /// <summary>Re-applies every static string and tooltip from Strings\&lt;lang&gt;\Resources.resw.</summary>
    public void Retranslate()
    {
        ConfigPanelTitle.Text = Strings.Get("ConfigPanelTitle");
        NetworkSettingsHeader.Text = Strings.Get("NetworkSettingsHeader");
        ReceiveIpPortLabel.Text = Strings.Get("ReceiveIpPortLabel");
        MulticastLabel.Text = Strings.Get("MulticastLabel");
        SendIpPortLabel.Text = Strings.Get("SendIpPortLabel");
        UsuallyLocalhostLabel.Text = Strings.Get("UsuallyLocalhostLabel");
        ResetToDefaultsButton.Content = Strings.Get("ResetToDefaultsButtonControl");
        FileLocationsHeader.Text = Strings.Get("FileLocationsHeader");
        JsonLocationLabel.Text = Strings.Get("JsonLocationLabel");
        BrowseButton.Content = Strings.Get("BrowseButtonControl");
        JsonLocationHint.Text = Strings.Get("JsonLocationHint");
        SaveButton.Content = Strings.Get("SaveButtonControl");
        CancelButton.Content = Strings.Get("CancelButtonControl");

        ToolTipService.SetToolTip(ReceiveIpTextBox, Strings.Get("Tooltip_ReceiveIp"));
        ToolTipService.SetToolTip(ReceivePortTextBox, Strings.Get("Tooltip_ReceivePort"));
        ToolTipService.SetToolTip(SendIpTextBox, Strings.Get("Tooltip_SendIp"));
        ToolTipService.SetToolTip(SendPortTextBox, Strings.Get("Tooltip_SendPort"));
        ToolTipService.SetToolTip(JsonLocationTextBox, Strings.Get("Tooltip_JsonLocation"));
    }

    /// <summary>Loads the given config into the form and waits until Save or Cancel is pressed.</summary>
    public Task<bool?> EditAsync(DcsBiosConfig config)
    {
        Config = new DcsBiosConfig
        {
            ReceiveFromIpUdp = config.ReceiveFromIpUdp,
            SendToIpUdp = config.SendToIpUdp,
            ReceivePortUdp = config.ReceivePortUdp,
            SendPortUdp = config.SendPortUdp,
            DcsBiosJsonLocation = config.DcsBiosJsonLocation
        };

        LoadConfigToUI();
        StatusText.Visibility = Visibility.Collapsed;

        _tcs = new TaskCompletionSource<bool?>();
        return _tcs.Task;
    }

    private void LoadConfigToUI()
    {
        ReceiveIpTextBox.Text = Config.ReceiveFromIpUdp;
        SendIpTextBox.Text = Config.SendToIpUdp;
        ReceivePortTextBox.Text = Config.ReceivePortUdp.ToString();
        SendPortTextBox.Text = Config.SendPortUdp.ToString();
        JsonLocationTextBox.Text = Config.DcsBiosJsonLocation;
    }

    private void SetStatus(string message, bool isError)
    {
        StatusText.Text = message;
        StatusText.Foreground = new SolidColorBrush(isError ? Colors.Red : Colors.Green);
        StatusText.Visibility = Visibility.Visible;
    }

    private bool ValidatePort(string portText, string portName, TextBox textBox, out int port)
    {
        if (!int.TryParse(portText, out port) || port is < MinPortNumber or > MaxPortNumber)
        {
            SetStatus(Strings.Format("Status_PortRangeFormat", portName, MinPortNumber, MaxPortNumber), true);
            textBox.Focus(FocusState.Programmatic);
            return false;
        }
        return true;
    }

    private bool ValidateAndUpdateConfig()
    {
        try
        {
            if (!IPAddress.TryParse(ReceiveIpTextBox.Text.Trim(), out var receiveIp))
            {
                SetStatus(Strings.Get("Status_ReceiveIpInvalid"), true);
                ReceiveIpTextBox.Focus(FocusState.Programmatic);
                return false;
            }

            if (!IPAddress.TryParse(SendIpTextBox.Text.Trim(), out var sendIp))
            {
                SetStatus(Strings.Get("Status_SendIpInvalid"), true);
                SendIpTextBox.Focus(FocusState.Programmatic);
                return false;
            }

            if (!ValidatePort(ReceivePortTextBox.Text.Trim(), Strings.Get("PortName_Receive"), ReceivePortTextBox, out int receivePort))
                return false;

            if (!ValidatePort(SendPortTextBox.Text.Trim(), Strings.Get("PortName_Send"), SendPortTextBox, out int sendPort))
                return false;

            string jsonLocation = JsonLocationTextBox.Text.Trim();
            if (string.IsNullOrWhiteSpace(jsonLocation))
            {
                SetStatus(Strings.Get("Status_JsonLocationEmpty"), true);
                JsonLocationTextBox.Focus(FocusState.Programmatic);
                return false;
            }

            if (!Directory.Exists(jsonLocation))
            {
                SetStatus(Strings.Format("Status_DirectoryNotExistFormat", jsonLocation), true);
                JsonLocationTextBox.Focus(FocusState.Programmatic);
                return false;
            }

            Config.ReceiveFromIpUdp = receiveIp.ToString();
            Config.SendToIpUdp = sendIp.ToString();
            Config.ReceivePortUdp = receivePort;
            Config.SendPortUdp = sendPort;
            Config.DcsBiosJsonLocation = jsonLocation;

            try
            {
                ConfigManager.Validate(Config);
            }
            catch (ConfigException ex)
            {
                SetStatus(ex.Message, true);
                return false;
            }

            var missing = ConfigManager.GetMissingExpectedJsonFiles(Config.DcsBiosJsonLocation);
            if (missing.Count > 0)
            {
                var files = string.Join(", ", missing);
                SetStatus(Strings.Format("Status_MissingJsonFilesFormat", files), true);
            }

            return true;
        }
        catch (Exception ex)
        {
            SetStatus(Strings.Format("Status_ValidationErrorFormat", ex.Message), true);
            return false;
        }
    }

    private async void BrowseButton_Click(object sender, RoutedEventArgs e)
    {
        try
        {
            var picker = new FolderPicker();
            InitializeWithWindow.Initialize(picker, OwnerHwnd);
            picker.FileTypeFilter.Add("*");

            var folder = await picker.PickSingleFolderAsync();
            if (folder != null)
            {
                JsonLocationTextBox.Text = folder.Path;
            }
        }
        catch (Exception ex)
        {
            App.Logger.Error(ex, "Folder picker failed");
            SetStatus(Strings.Format("Status_FolderPickerFailedFormat", ex.Message), true);
        }
    }

    private void SaveButton_Click(object sender, RoutedEventArgs e)
    {
        if (!ValidateAndUpdateConfig()) return;

        try
        {
            ConfigManager.Save(Config);
            _tcs?.TrySetResult(true);
        }
        catch (Exception ex)
        {
            SetStatus(Strings.Format("Status_SaveConfigFailedFormat", ex.Message), true);
        }
    }

    private void CancelButton_Click(object sender, RoutedEventArgs e)
    {
        _tcs?.TrySetResult(false);
    }

    private void ResetToDefaultsButton_Click(object sender, RoutedEventArgs e)
    {
        Config = new DcsBiosConfig();
        LoadConfigToUI();
        SetStatus(Strings.Get("Status_SettingsResetToDefaults"), false);
    }
}
