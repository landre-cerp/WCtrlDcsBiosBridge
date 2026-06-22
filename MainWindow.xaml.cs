using NLog;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using WwDevicesDotNet;
using WCtrlDcsBiosBridge.Config;
using System.Diagnostics;
using WCtrlDcsBiosBridge.Devices;
using WCtrlDcsBiosBridge.Services;
using WCtrlDcsBiosBridge.Aircrafts;
using Microsoft.Win32;
using System.Windows.Navigation;

namespace WCtrlDcsBiosBridge;

public partial class MainWindow : Window, IDisposable, INotifyPropertyChanged
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private DcsBiosConfig config = new();
    private UserOptions userOptions = new();
    private readonly List<DeviceInfo> devices = new();

    private bool NeedsConfigEdit => !IsConfigValid();

    private bool _disposed = false;
    private BridgeManager? bridgeManager;
    private CancellationTokenSource? _detectCts;
    private readonly CancellationTokenSource _shutdownCts = new();
    private ThemePreference _currentTheme = ThemePreference.System;

    private const string GitHubOwner = "landre-cerp";
    private const string GitHubRepo = "WCtrlDcsBiosBridge";

    // Dedicated update notification state
    private string? _updateMessage;
    private string? _updateUrl;
    private bool _isUpdateVisible;

    // Update service
    private readonly GitHubUpdateService _updateService = new(GitHubOwner, GitHubRepo);

    public string? UpdateMessage { get => _updateMessage; private set { _updateMessage = value; OnPropertyChanged(); } }
    public string? UpdateUrl { get => _updateUrl; private set { _updateUrl = value; OnPropertyChanged(); } }
    public bool IsUpdateVisible { get => _isUpdateVisible; private set { _isUpdateVisible = value; OnPropertyChanged(); } }

    public bool IsBridgeRunning => bridgeManager?.IsStarted == true;
    public bool CanEdit => !IsBridgeRunning;

    // Null while idle or detecting; set to the active descriptor once an aircraft is confirmed.
    private AircraftDescriptor? _detectedAircraft;

    // Each section is editable unless its specific aircraft is currently running.
    public bool IsA10COptionsEnabled   => _detectedAircraft != AircraftRegistry.A10C;
    public bool IsFA18COptionsEnabled  => _detectedAircraft != AircraftRegistry.FA18C;
    public bool IsF14BOptionsEnabled   => _detectedAircraft != AircraftRegistry.F14B;
    public bool IsF16COptionsEnabled   => _detectedAircraft != AircraftRegistry.F16C;
    public bool IsLightingManaged      => !userOptions.DisableLightingManagement;

    public string AppVersion { get; }

    // Live DCS-BIOS exporter version reported by DCS; null until received.
    private string? _dcsBiosVersion;

    public event PropertyChangedEventHandler? PropertyChanged;
    private void OnPropertyChanged([CallerMemberName] string? name = null) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

    public MainWindow()
    {
        SetupLogging();
        InitializeComponent();

        AppVersion = AppVersionProvider.GetAppVersion();
        UpdateTitle();

        OptionsPanel.SettingsChanged += (sender, args) =>
        {
            SaveUserSettings();
            OnPropertyChanged(nameof(IsLightingManaged));
        };

        LoadConfig();
        LoadUserSettings();
        _currentTheme = userOptions.Theme;
        UpdateThemeToggleIcon();

        SystemEvents.UserPreferenceChanged += OnSystemPreferenceChanged;

        _ = DetectDevicesAsync();
        UpdateState();
        Loaded += MainWindow_Loaded;
    }

    private bool IsConfigValid() => !string.IsNullOrWhiteSpace(config.DcsBiosJsonLocation);

    private async Task DetectDevicesAsync()
    {
        _detectCts?.Cancel();
        _detectCts = new CancellationTokenSource();
        devices.Clear();
        ShowStatus("Detecting devices...", false);

        try
        {
            var progress = new Progress<DeviceManager.DeviceDetectionProgress>(p =>
            {
                ShowStatus(p.Message, false);
            });
            var detected = await DeviceManager.DetectAndConnectDevicesAsync(progress, _detectCts.Token,
                resetDevices: !userOptions.DisableLightingManagement);
            devices.AddRange(detected);
            BuildDeviceUI();
            UpdateStartButtonState();
            
            if (CanStartBridge() && userOptions.AutoStart)
            {
                Logger.Info("Auto-starting bridge...");
                await StartBridge();
            }
        }
        catch (OperationCanceledException)
        {
            ShowStatus("Device detection cancelled", true);
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Async device detection failed");
            ShowStatus($"Device detection failed: {ex.Message}", true);
        }
    }

    private bool CanStartBridge()
    {
        return IsConfigValid() && devices.Count > 0 && !IsBridgeRunning;
    }

    private void BuildDeviceUI()
    {
        if (devices.Count == 0)
        {
            ShowStatus("No devices detected. Please ensure your device is connected.", true);
            return;
        }
        try
        {
            var cduCount = devices.Count(d => d.Cdu != null);
            var frontpanelCount = devices.Count(d => d.Frontpanel != null);

            bool multipleDevices = devices.Count > 1;
            DeviceListPanel.Children.Clear();

            foreach (var deviceInfo in devices)
            {
                DeviceListPanel.Children.Add(CreateDeviceCard(deviceInfo));

            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to build device UI");
            ShowStatus($"Failed to build device UI: {ex.Message}", true);
        }
    }

    private static readonly SolidColorBrush CardDisconnectedBg     = new(Color.FromArgb(0x18, 0xC6, 0x28, 0x28));
    private static readonly SolidColorBrush CardDisconnectedDot    = new(Color.FromRgb(0xE5, 0x39, 0x35));
    private static readonly SolidColorBrush CardDisconnectedBorder = new(Color.FromRgb(0xB7, 0x1C, 0x1C));

    private Border CreateDeviceCard(DeviceInfo deviceInfo)
    {
        var dot = new Ellipse
        {
            Width = 7,
            Height = 7,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(0, 0, 7, 0)
        };
        dot.SetResourceReference(Shape.FillProperty, "CardConnectedDotBrush");

        var label = new TextBlock
        {
            Text = deviceInfo.DisplayName,
            FontSize = 12,
            VerticalAlignment = VerticalAlignment.Center
        };

        var inner = new StackPanel { Orientation = Orientation.Horizontal };
        inner.Children.Add(dot);
        inner.Children.Add(label);

        var card = new Border
        {
            CornerRadius = new CornerRadius(6),
            BorderThickness = new Thickness(1),
            Padding = new Thickness(10, 5, 10, 5),
            Margin = new Thickness(0, 0, 8, 6),
            Child = inner
        };
        card.SetResourceReference(Border.BorderBrushProperty, "CardConnectedBorderBrush");
        card.SetResourceReference(Border.BackgroundProperty,  "CardConnectedBgBrush");

        if (deviceInfo.Frontpanel != null)
        {
            deviceInfo.Frontpanel.Disconnected += (s, e) =>
            {
                Application.Current.Dispatcher.Invoke(() =>
                {
                    dot.Fill = CardDisconnectedDot;
                    card.Background = CardDisconnectedBg;
                    card.BorderBrush = CardDisconnectedBorder;
                });
            };
        }

        return card;
    }

    private void UpdateState()
    {
        UpdateStartButtonState();
    }

    private async void MainWindow_Loaded(object? sender, RoutedEventArgs e)
    {
        if (NeedsConfigEdit)
        {
            OpenConfigEditor();
        }

        // Bind the OptionsPanel to the userOptions object
        OptionsPanel.DataContext = userOptions;

        try
        {
            await CheckForUpdatesAndNotifyAsync();
        }
        catch (Exception ex)
        {
            Logger.Debug(ex, "Failed to check GitHub for latest release");
        }
    }

    private void SetupLogging() => LogManager.ThrowConfigExceptions = true;

    private void LoadConfig()
    {
        var result = ConfigManager.TryLoad();
        result.Match(
            onSuccess: cfg =>
            {
                config = cfg;
                Logger.Info("Configuration loaded successfully.");
                return 0; // Unit equivalent
            },
            onFailure: error =>
            {
                ShowStatus(error, true);
                Logger.Warn($"Configuration load failed: {error}");
                return 0;
            }
        );
    }

    private void ConfigButton_Click(object sender, RoutedEventArgs e)
    {
        if (IsBridgeRunning)
        {
            ShowStatus("Cannot edit DCS-BIOS configuration while bridge is running.", true);
            return;
        }
        OpenConfigEditor();
    }

    private void OpenConfigEditor()
    {
        try
        {
            var configWindow = new ConfigWindow(config);
            configWindow.Owner = this;

            if (configWindow.ShowDialog() == true)
            {
                config = configWindow.Config;
                UpdateState();

                if (IsConfigValid())
                {
                    ShowStatus("Configuration loaded. Ready to start bridge.", false);
                }
                else
                {
                    ShowStatus("Please edit DCS-BIOS config", true);
                }

                if (IsBridgeRunning)
                {
                    ShowStatus("Configuration updated. Please restart the bridge for changes to take effect.", false);
                }
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to open configuration editor");
            ShowStatus($"Failed to open configuration editor: {ex.Message}", true);
        }
    }

    private async void StartButton_Click(object sender, RoutedEventArgs e)
    {
        await StartBridge();
    }

    private async Task StartBridge()
    {
        if (!IsConfigValid())
        {
            ShowStatus("Configuration not loaded. Please check the configuration settings.", true);
            return;
        }

        if (devices.Count == 0)
        {
            ShowStatus("No devices found. Please ensure your device is connected and refresh.", true);
            await DetectDevicesAsync();
            UpdateState();
            if (devices.Count == 0) return;
        }

        SaveUserSettings();

        // Clear any previous error messages when starting
        ShowStatus("Starting bridge...", false);

        StartButton.IsEnabled = false;
        StartButton.Content = "Starting...";

        try
        {
            bridgeManager = new BridgeManager();
            bridgeManager.DetectedAircraftChanged += OnDetectedAircraftChanged;
            bridgeManager.DcsBiosVersionChanged += OnDcsBiosVersionChanged;
            OnPropertyChanged(nameof(IsBridgeRunning));
            OnPropertyChanged(nameof(CanEdit));

            // StartAsync runs the detection loop and never returns until
            // cancelled or an error occurs.
            await bridgeManager.StartAsync(devices, userOptions, config, _shutdownCts.Token);
        }
        catch (OperationCanceledException)
        {
            Logger.Info("Bridge cancelled by application shutdown");
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to start bridge");
            ShowStatus($"Failed to start bridge: {ex.Message}", true);
            ResetStartButton();
        }
    }

    private void OnDetectedAircraftChanged(string? dcsBiosName)
    {
        Dispatcher.Invoke(() =>
        {
            if (dcsBiosName == null)
            {
                SetDetectedAircraft(null);
                ShowStatus("Waiting for DCS aircraft...", false);
                StartButton.Content = "Detecting...";
                UpdateBridgeStatusCard("Waiting for DCS aircraft...", "Load a supported module in DCS World");
            }
            else if (AircraftRegistry.FindByDcsBiosName(dcsBiosName) is not { } descriptor)
            {
                SetDetectedAircraft(null);
                ShowStatus($"Unsupported aircraft: {dcsBiosName} — waiting...", false);
                StartButton.Content = "Detecting...";
                UpdateBridgeStatusCard($"Unsupported aircraft: {dcsBiosName}", "Waiting for a supported module...");
            }
            else
            {
                SetDetectedAircraft(descriptor);
                ShowStatus($"Detected: {descriptor.DisplayName}", false);
                StartButton.Content = "Bridge Running";
                StartButton.IsEnabled = false;
                OnPropertyChanged(nameof(IsBridgeRunning));
                OnPropertyChanged(nameof(CanEdit));
                UpdateBridgeStatusCard($"Bridge running · {descriptor.DisplayName}", $"Listening on {config.ReceiveFromIpUdp}:{config.ReceivePortUdp}");

                if (userOptions.MinimizeOnStart)
                {
                    WindowState = WindowState.Minimized;
                }
            }
        });
    }

    private void UpdateBridgeStatusCard(string label, string sub)
    {
        BridgeStatusDot.Fill = IsBridgeRunning ? Brushes.Green : new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));
        BridgeStatusLabel.Text = label;
        BridgeStatusSub.Text = sub;
    }

    private void SetDetectedAircraft(AircraftDescriptor? descriptor)
    {
        _detectedAircraft = descriptor;
        OnPropertyChanged(nameof(IsA10COptionsEnabled));
        OnPropertyChanged(nameof(IsFA18COptionsEnabled));
        OnPropertyChanged(nameof(IsF14BOptionsEnabled));
        OnPropertyChanged(nameof(IsF16COptionsEnabled));
    }

    private void ResetStartButton()
    {
        SetDetectedAircraft(null);
        StartButton.IsEnabled = !IsBridgeRunning && devices.Count > 0 && IsConfigValid();
        StartButton.Content = "Start Bridge";
        _dcsBiosVersion = null;
        UpdateTitle();
        OnPropertyChanged(nameof(IsBridgeRunning));
        OnPropertyChanged(nameof(CanEdit));
        UpdateBridgeStatusCard("Bridge stopped", "Aircraft is detected automatically when DCS is running");
    }

    private void OnDcsBiosVersionChanged(string? dcsBiosVersion)
    {
        Dispatcher.Invoke(() =>
        {
            _dcsBiosVersion = dcsBiosVersion;
            UpdateTitle();
        });
    }

    private void UpdateTitle()
    {
        var dcsBios = string.IsNullOrEmpty(_dcsBiosVersion) ? "--.--" : _dcsBiosVersion;
        Title = $"WctrlDcsBiosBridge v{AppVersion} | DCS-BIOS {dcsBios}";
    }

    private void OnSystemPreferenceChanged(object sender, UserPreferenceChangedEventArgs e)
    {
        if (e.Category == UserPreferenceCategory.General)
        {
            Dispatcher.Invoke(() => ThemeManager.Apply(_currentTheme));
        }
    }

    private void LoadUserSettings()
    {
        var result = UserOptionsStorage.TryLoad();
        if (result.IsSuccess)
        {
            userOptions = result.Value!;
        }
        else
        {
            Logger.Warn($"Failed to load user options: {result.Error}");
            userOptions = UserOptionsStorage.GetDefaultOptions();
        }
    }
    private void SaveUserSettings() => UserOptionsStorage.Save(userOptions);

    private void ShowStatus(string message, bool isError)
    {
        StatusControl.ShowStatus(message, isError);
    }

    private void UpdateStartButtonState()
    {
        StartButton.IsEnabled = !IsBridgeRunning && IsConfigValid() && devices.Count > 0;
        if (!IsBridgeRunning && !(StartButton.Content?.ToString()?.Length > 0))
        {
            StartButton.Content = "Start Bridge";
        }
    }

    private void ThemeToggle_Click(object sender, RoutedEventArgs e)
    {
        _currentTheme = _currentTheme switch
        {
            ThemePreference.System => ThemePreference.Light,
            ThemePreference.Light  => ThemePreference.Dark,
            ThemePreference.Dark   => ThemePreference.DCS,
            _                      => ThemePreference.System
        };
        ThemeManager.Apply(_currentTheme);
        userOptions.Theme = _currentTheme;
        SaveUserSettings();
        UpdateThemeToggleIcon();
    }

    private void UpdateThemeToggleIcon()
    {
        ThemeIcon.Text = _currentTheme switch
        {
            ThemePreference.Light => "☀",
            ThemePreference.Dark  => "☾",
            ThemePreference.DCS   => "✈",
            _                     => "◑"
        };
        ThemeToggleButton.ToolTip = _currentTheme switch
        {
            ThemePreference.Light => "Theme: Light — click for Dark",
            ThemePreference.Dark  => "Theme: Dark — click for DCS",
            ThemePreference.DCS   => "Theme: DCS — click for System",
            _                     => "Theme: System — click for Light"
        };
    }

    protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
    {
        _shutdownCts.Cancel();

        if (bridgeManager != null)
        {
            try
            {
                if (IsBridgeRunning) bridgeManager.Stop();
                bridgeManager.DcsBiosVersionChanged -= OnDcsBiosVersionChanged;
                bridgeManager.Dispose();
                bridgeManager = null;
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error stopping bridge during close");
            }
        }

        base.OnClosing(e);
    }

    protected override void OnClosed(EventArgs e)
    {
        if (!_disposed)
        {
            Dispose();
        }
        base.OnClosed(e);
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
        if (_disposed)
            return;

        if (disposing)
        {
            SystemEvents.UserPreferenceChanged -= OnSystemPreferenceChanged;
            _shutdownCts.Cancel();
            _detectCts?.Cancel();
            bridgeManager?.Dispose();
            DeviceManager.DisposeDevices(devices,
                userOptions.DisableLightingManagement ? null : CloseResetOptions.From(userOptions));
            SaveUserSettings();
        }

        _disposed = true;
    }

    private async Task CheckForUpdatesAndNotifyAsync()
    {
        try
        {
            var channel = AppVersionProvider.IsPreRelease(AppVersion) ? UpdateChannel.Prerelease : UpdateChannel.Stable;
            var result = await _updateService.CheckForUpdatesAsync(AppVersion, channel);
            if (result is { HasUpdate: true })
            {
                SetUpdateNotification($"New version available: {result.LatestTag}", result.HtmlUrl);
                Logger.Info($"New release available: {result.LatestTag} - {result.HtmlUrl}");
            }
        }
        catch (Exception ex)
        {
            Logger.Debug(ex, "Failed to check GitHub for latest release");
        }
    }

    private void SetUpdateNotification(string message, string? url)
    {
        UpdateMessage = message;
        UpdateUrl = url;
        IsUpdateVisible = true;
    }

    private void DismissUpdate_Click(object sender, RoutedEventArgs e)
    {
        IsUpdateVisible = false;
    }

    private void OpenUpdateLink_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrWhiteSpace(UpdateUrl)) return;
        try
        {
            Process.Start(new ProcessStartInfo(UpdateUrl) { UseShellExecute = true });
        }
        catch (Exception ex)
        {
            Logger.Warn(ex, "Failed to open update URL");
        }
    }

    private void DocsLink_RequestNavigate(object sender, RequestNavigateEventArgs e)
    {
        try
        {
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri) { UseShellExecute = true });
        }
        catch (Exception ex)
        {
            Logger.Warn(ex, "Failed to open docs URL");
        }
        e.Handled = true;
    }
}
