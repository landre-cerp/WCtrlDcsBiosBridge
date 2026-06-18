using Microsoft.Win32;
using System;
using System.IO;
using System.Net;
using System.Windows;

namespace WCtrlDcsBiosBridge.Config;

public partial class ConfigWindow : Window
{
    private const int MinPortNumber = 1;
    private const int MaxPortNumber = 65535;

    public DcsBiosConfig Config { get; private set; }

    public ConfigWindow(DcsBiosConfig config)
    {
        InitializeComponent();
        Config = new DcsBiosConfig
        {
            ReceiveFromIpUdp = config.ReceiveFromIpUdp,
            SendToIpUdp = config.SendToIpUdp,
            ReceivePortUdp = config.ReceivePortUdp,
            SendPortUdp = config.SendPortUdp,
            DcsBiosJsonLocation = config.DcsBiosJsonLocation
        };
        
        LoadConfigToUI();
    }

    private void LoadConfigToUI()
    {
        ReceiveIpTextBox.Text = Config.ReceiveFromIpUdp;
        SendIpTextBox.Text = Config.SendToIpUdp;
        ReceivePortTextBox.Text = Config.ReceivePortUdp.ToString();
        SendPortTextBox.Text = Config.SendPortUdp.ToString();
        JsonLocationTextBox.Text = Config.DcsBiosJsonLocation;
    }

    private static void ShowValidationError(string message, System.Windows.Controls.Control controlToFocus)
    {
        MessageBox.Show(message, "Validation Error", MessageBoxButton.OK, MessageBoxImage.Warning);
        controlToFocus.Focus();
    }

    private bool ValidatePort(string portText, string portName, System.Windows.Controls.TextBox textBox, out int port)
    {
        if (!int.TryParse(portText, out port) || port is < MinPortNumber or > MaxPortNumber)
        {
            ShowValidationError($"{portName} must be between {MinPortNumber} and {MaxPortNumber}.", textBox);
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
                ShowValidationError("Receive IP address is not valid.", ReceiveIpTextBox);
                return false;
            }

            if (!IPAddress.TryParse(SendIpTextBox.Text.Trim(), out var sendIp))
            {
                ShowValidationError("Send IP address is not valid.", SendIpTextBox);
                return false;
            }

            if (!ValidatePort(ReceivePortTextBox.Text.Trim(), "Receive port", ReceivePortTextBox, out int receivePort))
                return false;

            if (!ValidatePort(SendPortTextBox.Text.Trim(), "Send port", SendPortTextBox, out int sendPort))
                return false;
            
            string jsonLocation = JsonLocationTextBox.Text.Trim();
            if (string.IsNullOrWhiteSpace(jsonLocation))
            {
                ShowValidationError("DCS-BIOS JSON location cannot be empty.", JsonLocationTextBox); 
                return false;
            }

            if (!Directory.Exists(jsonLocation))
            {
                ShowValidationError($"The specified directory does not exist: {jsonLocation}", JsonLocationTextBox); 
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
                ShowValidationError(ex.Message, JsonLocationTextBox);
                return false;
            }

            var missing = ConfigManager.GetMissingExpectedJsonFiles(Config.DcsBiosJsonLocation);
            if (missing.Count > 0)
            {
                var files = string.Join(Environment.NewLine + " - ", missing);
                MessageBox.Show(
                    "Some expected DCS-BIOS JSON files are missing. You can continue, but related aircraft may not work until these files are present:" +
                    Environment.NewLine + " - " + files,
                    "Missing JSON Files",
                    MessageBoxButton.OK,
                    MessageBoxImage.Warning);
            }

            return true;
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Validation error: {ex.Message}", "Validation Error", 
                            MessageBoxButton.OK, MessageBoxImage.Error);
            return false;
        }
    }

    private void BrowseButton_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new OpenFolderDialog
        {
            Title = "Select DCS-BIOS JSON Files Directory"
        };

        if (!string.IsNullOrWhiteSpace(JsonLocationTextBox.Text) && Directory.Exists(JsonLocationTextBox.Text))
        {
            dialog.InitialDirectory = JsonLocationTextBox.Text;
        }

        if (dialog.ShowDialog() == true)
        {
            JsonLocationTextBox.Text = dialog.FolderName;
        }
    }

    private void SaveButton_Click(object sender, RoutedEventArgs e)
    {
        if (!ValidateAndUpdateConfig()) return;

        try
        {
            ConfigManager.Save(Config);
            MessageBox.Show("Configuration saved successfully!", "Success", 
                            MessageBoxButton.OK, MessageBoxImage.Information);
            DialogResult = true;
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to save configuration: {ex.Message}", "Save Error", 
                            MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void CancelButton_Click(object sender, RoutedEventArgs e)
    {
        DialogResult = false;
    }

    private void ResetToDefaultsButton_Click(object sender, RoutedEventArgs e)
    {
        var result = MessageBox.Show("Are you sure you want to reset all settings to default values?", 
                                    "Reset to Defaults", 
                                    MessageBoxButton.YesNo, 
                                    MessageBoxImage.Question);
        
        if (result == MessageBoxResult.Yes)
        {
            Config = new DcsBiosConfig();
            LoadConfigToUI();
        }
    }
}