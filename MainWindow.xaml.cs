using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Microsoft.UI;
using Microsoft.UI.Windowing;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using Microsoft.UI.Xaml.Shapes;
using NLog;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Devices;
using WCtrlDcsBiosBridge.Services;
using Windows.UI.ViewManagement;
using WinRT.Interop;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge;

public partial class MainWindow : Window, IDisposable, INotifyPropertyChanged
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private DcsBiosConfig config = new();
    private UserOptions userOptions = new();
    private readonly List<DeviceInfo> devices = new();

    private DeviceWatcher? _deviceWatcher;
    private readonly Dictionary<DeviceIdentifier, Border> _deviceCards = new();

    private bool NeedsConfigEdit => !IsConfigValid();

    private bool _disposed = false;
    private BridgeManager? bridgeManager;
    private CancellationTokenSource? _detectCts;
    private readonly CancellationTokenSource _shutdownCts = new();
    private ThemePreference _currentTheme = ThemePreference.System;

    private const string GitHubOwner = "landre-cerp";
    private const string GitHubRepo = "WCtrlDcsBiosBridge";

    private string? _updateMessage;
    private string? _updateUrl;
    private bool _isUpdateVisible;

    private readonly GitHubUpdateService _updateService = new(GitHubOwner, GitHubRepo);

    private readonly AppWindow _appWindow;
    private readonly UISettings _uiSettings = new();

    public string? UpdateMessage { get => _updateMessage; private set { _updateMessage = value; OnPropertyChanged(); } }
    public string? UpdateUrl { get => _updateUrl; private set { _updateUrl = value; OnPropertyChanged(); } }
    public bool IsUpdateVisible { get => _isUpdateVisible; private set { _isUpdateVisible = value; OnPropertyChanged(); } }

    public bool IsBridgeRunning => bridgeManager?.IsStarted == true;

    public bool IsBridgeLoopActive => bridgeManager?.IsLoopActive == true;

    public bool CanEdit => !IsBridgeLoopActive;

    private AircraftDescriptor? _detectedAircraft;

    public string? DetectedAircraftName => _detectedAircraft?.DisplayName;
    public bool IsLightingManaged => !userOptions.DisableLightingManagement;

    public string AppVersion { get; }

    private string? _dcsBiosVersion;

    public event PropertyChangedEventHandler? PropertyChanged;
    private void OnPropertyChanged([CallerMemberName] string? name = null) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

    public MainWindow()
    {
        SetupLogging();
        InitializeComponent();
        RootGrid.DataContext = this;

        var hwnd = WindowNative.GetWindowHandle(this);
        var windowId = Win32Interop.GetWindowIdFromWindow(hwnd);
        _appWindow = AppWindow.GetFromWindowId(windowId);
        ConfigureWindow();
        ConfigPanelControl.OwnerHwnd = hwnd;

        AppVersion = AppVersionProvider.GetAppVersion();
        UpdateTitle();

        OptionsPanel.SettingsChanged += (sender, args) =>
        {
            SaveUserSettings();
            OnPropertyChanged(nameof(IsLightingManaged));
            OptionsPanel.SetLightingManaged(IsLightingManaged);
        };

        LoadConfig();
        LoadUserSettings();
        _currentTheme = userOptions.Theme;
        WindowRoot.RequestedTheme = ThemeManager.CurrentElementTheme;
        UpdateThemeToggleIcon();
        OptionsPanel.SetLightingManaged(IsLightingManaged);

        _uiSettings.ColorValuesChanged += OnSystemPreferenceChanged;

        _ = DetectDevicesAsync();
        UpdateState();
        RootGrid.Loaded += MainWindow_Loaded;
        Closed += MainWindow_Closed;
    }

    private void ConfigureWindow()
    {
        var hwnd = WindowNative.GetWindowHandle(this);
        WindowSizing.Resize(_appWindow, hwnd, 700, 680);
        if (_appWindow.Presenter is OverlappedPresenter presenter)
        {
            presenter.IsResizable = true;
        }

        WindowSizing.CenterOnDisplay(_appWindow);
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

            StartDeviceWatcher();

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
        return IsConfigValid() && devices.Count > 0 && !IsBridgeLoopActive;
    }

    private void StartDeviceWatcher()
    {
        if (_deviceWatcher != null) return;

        _deviceWatcher = new DeviceWatcher();
        _deviceWatcher.DeviceArrived += id => DispatcherQueue.TryEnqueue(() => _ = OnDeviceArrivedAsync(id));
        _deviceWatcher.DeviceRemoved += id => DispatcherQueue.TryEnqueue(() => OnDeviceRemoved(id));
        _deviceWatcher.Start();
    }

    private async Task OnDeviceArrivedAsync(DeviceIdentifier deviceId)
    {
        if (devices.Any(d => d.DeviceId.Equals(deviceId)))
            return;

        ShowStatus($"Connecting {DeviceManager.GetDeviceName(deviceId)}...", false);
        try
        {
            var info = await DeviceManager.ConnectDeviceAsync(deviceId,
                resetDevices: !userOptions.DisableLightingManagement);
            if (info == null)
            {
                ShowStatus($"Failed to connect {deviceId.Description}", true);
                return;
            }

            devices.Add(info);
            AddDeviceCard(info);

            if (bridgeManager?.IsLoopActive == true)
                bridgeManager.AddDevice(info);

            UpdateStartButtonState();
            ShowStatus($"Connected {info.DisplayName}", false);

            if (CanStartBridge() && userOptions.AutoStart)
            {
                Logger.Info("Auto-starting bridge after device arrival...");
                await StartBridge();
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, $"Failed to hot-connect device {deviceId.Description}");
            ShowStatus($"Failed to connect {deviceId.Description}: {ex.Message}", true);
        }
    }

    private void OnDeviceRemoved(DeviceIdentifier deviceId)
    {
        var info = devices.FirstOrDefault(d => d.DeviceId.Equals(deviceId));
        if (info == null) return;

        if (bridgeManager?.IsLoopActive == true)
            bridgeManager.RemoveDevice(deviceId);

        devices.Remove(info);
        RemoveDeviceCard(deviceId);

        try { info.Dispose(); }
        catch (Exception ex) { Logger.Warn(ex, "Error disposing removed device"); }

        UpdateStartButtonState();
        ShowStatus($"Disconnected {info.DisplayName}", false);
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
            DeviceListPanel.Children.Clear();
            _deviceCards.Clear();

            foreach (var deviceInfo in devices)
                AddDeviceCard(deviceInfo);
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to build device UI");
            ShowStatus($"Failed to build device UI: {ex.Message}", true);
        }
    }

    private void AddDeviceCard(DeviceInfo deviceInfo)
    {
        var card = CreateDeviceCard(deviceInfo);
        DeviceListPanel.Children.Add(card);
        _deviceCards[deviceInfo.DeviceId] = card;
    }

    private void RemoveDeviceCard(DeviceIdentifier deviceId)
    {
        if (_deviceCards.TryGetValue(deviceId, out var card))
        {
            DeviceListPanel.Children.Remove(card);
            _deviceCards.Remove(deviceId);
        }
    }

    private static readonly SolidColorBrush CardDisconnectedBg = new(Windows.UI.Color.FromArgb(0x18, 0xC6, 0x28, 0x28));
    private static readonly SolidColorBrush CardDisconnectedDot = new(Windows.UI.Color.FromArgb(0xFF, 0xE5, 0x39, 0x35));
    private static readonly SolidColorBrush CardDisconnectedBorder = new(Windows.UI.Color.FromArgb(0xFF, 0xB7, 0x1C, 0x1C));

    private Border CreateDeviceCard(DeviceInfo deviceInfo)
    {
        var dot = new Ellipse
        {
            Width = 7,
            Height = 7,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(0, 0, 7, 0),
            Fill = (Brush)Application.Current.Resources["CardConnectedDotBrush"]
        };

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
            Child = inner,
            BorderBrush = (Brush)Application.Current.Resources["CardConnectedBorderBrush"],
            Background = (Brush)Application.Current.Resources["CardConnectedBgBrush"]
        };

        void OnDisconnected(object? s, EventArgs e) =>
            DispatcherQueue.TryEnqueue(() =>
            {
                dot.Fill = CardDisconnectedDot;
                card.Background = CardDisconnectedBg;
                card.BorderBrush = CardDisconnectedBorder;
            });

        if (deviceInfo.Cdu != null)
            deviceInfo.Cdu.Disconnected += OnDisconnected;
        else if (deviceInfo.Frontpanel != null)
            deviceInfo.Frontpanel.Disconnected += OnDisconnected;

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
            await OpenConfigEditor();
        }

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
                return 0;
            },
            onFailure: error =>
            {
                ShowStatus(error, true);
                Logger.Warn($"Configuration load failed: {error}");
                return 0;
            }
        );
    }

    private async void ConfigButton_Click(object sender, RoutedEventArgs e)
    {
        if (IsBridgeRunning)
        {
            ShowStatus("Cannot edit DCS-BIOS configuration while bridge is running.", true);
            return;
        }
        await OpenConfigEditor();
    }

    private async Task OpenConfigEditor()
    {
        try
        {
            ConfigOverlay.Visibility = Visibility.Visible;
            var result = await ConfigPanelControl.EditAsync(config);
            ConfigOverlay.Visibility = Visibility.Collapsed;

            if (result == true)
            {
                config = ConfigPanelControl.Config;
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

        ShowStatus("Starting bridge...", false);

        StartButton.IsEnabled = false;
        StartButton.Content = "Starting...";

        try
        {
            bridgeManager = new BridgeManager();
            bridgeManager.DetectedAircraftChanged += OnDetectedAircraftChanged;
            bridgeManager.DcsBiosVersionChanged += OnDcsBiosVersionChanged;
            OnPropertyChanged(nameof(IsBridgeRunning));
            OnPropertyChanged(nameof(IsBridgeLoopActive));
            OnPropertyChanged(nameof(CanEdit));

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
        DispatcherQueue.TryEnqueue(() =>
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
                OnPropertyChanged(nameof(IsBridgeLoopActive));
                OnPropertyChanged(nameof(CanEdit));
                UpdateBridgeStatusCard($"Bridge running · {descriptor.DisplayName}", $"Listening on {config.ReceiveFromIpUdp}:{config.ReceivePortUdp}");

                if (userOptions.MinimizeOnStart && _appWindow.Presenter is OverlappedPresenter p)
                {
                    p.Minimize();
                }
            }
        });
    }

    private void UpdateBridgeStatusCard(string label, string sub)
    {
        BridgeStatusDot.Fill = IsBridgeRunning
            ? new SolidColorBrush(Colors.Green)
            : new SolidColorBrush(Windows.UI.Color.FromArgb(0xFF, 0x88, 0x88, 0x88));
        BridgeStatusLabel.Text = label;
        BridgeStatusSub.Text = sub;
    }

    private void SetDetectedAircraft(AircraftDescriptor? descriptor)
    {
        _detectedAircraft = descriptor;
        OnPropertyChanged(nameof(DetectedAircraftName));
        OptionsPanel.SetDetectedAircraft(DetectedAircraftName);
    }

    private void ResetStartButton()
    {
        SetDetectedAircraft(null);
        StartButton.IsEnabled = !IsBridgeLoopActive && devices.Count > 0 && IsConfigValid();
        StartButton.Content = "Start Bridge";
        _dcsBiosVersion = null;
        UpdateTitle();
        OnPropertyChanged(nameof(IsBridgeRunning));
        OnPropertyChanged(nameof(IsBridgeLoopActive));
        OnPropertyChanged(nameof(CanEdit));
        UpdateBridgeStatusCard("Bridge stopped", "Aircraft is detected automatically when DCS is running");
    }

    private void OnDcsBiosVersionChanged(string? dcsBiosVersion)
    {
        DispatcherQueue.TryEnqueue(() =>
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

    private void OnSystemPreferenceChanged(UISettings sender, object args)
    {
        DispatcherQueue.TryEnqueue(() =>
        {
            ThemeManager.Apply(_currentTheme);
            WindowRoot.RequestedTheme = ThemeManager.CurrentElementTheme;
        });
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
        StartButton.IsEnabled = !IsBridgeLoopActive && IsConfigValid() && devices.Count > 0;
        if (!IsBridgeLoopActive && !(StartButton.Content is string s && s.Length > 0))
        {
            StartButton.Content = "Start Bridge";
        }
    }

    private void ThemeToggle_Click(object sender, RoutedEventArgs e)
    {
        _currentTheme = _currentTheme switch
        {
            ThemePreference.System => ThemePreference.Light,
            ThemePreference.Light => ThemePreference.Dark,
            _ => ThemePreference.System
        };
        ThemeManager.Apply(_currentTheme);
        WindowRoot.RequestedTheme = ThemeManager.CurrentElementTheme;
        userOptions.Theme = _currentTheme;
        SaveUserSettings();
        UpdateThemeToggleIcon();
    }

    private void UpdateThemeToggleIcon()
    {
        // ThemePreference.DCS is treated the same as Dark (folded into the default
        // dark palette) — it can still show up here from an old persisted config.json.
        ThemeIcon.Text = _currentTheme switch
        {
            ThemePreference.Light => "☀",
            ThemePreference.Dark or ThemePreference.DCS => "☾",
            _ => "◑"
        };
        ToolTipService.SetToolTip(ThemeToggleButton, _currentTheme switch
        {
            ThemePreference.Light => "Theme: Light — click for Dark",
            ThemePreference.Dark or ThemePreference.DCS => "Theme: Dark — click for System",
            _ => "Theme: System — click for Light"
        });
    }

    private void MainWindow_Closed(object sender, WindowEventArgs e)
    {
        _shutdownCts.Cancel();

        try { _deviceWatcher?.Dispose(); _deviceWatcher = null; }
        catch (Exception ex) { Logger.Warn(ex, "Error disposing device watcher during close"); }

        if (bridgeManager != null)
        {
            try
            {
                if (bridgeManager.IsLoopActive) bridgeManager.Stop();
                bridgeManager.DcsBiosVersionChanged -= OnDcsBiosVersionChanged;
                bridgeManager.Dispose();
                bridgeManager = null;
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error stopping bridge during close");
            }
        }

        if (!_disposed)
        {
            Dispose();
        }
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
            _uiSettings.ColorValuesChanged -= OnSystemPreferenceChanged;
            _shutdownCts.Cancel();
            _detectCts?.Cancel();
            _deviceWatcher?.Dispose();
            _deviceWatcher = null;
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

    private void DocsLink_Click(Microsoft.UI.Xaml.Documents.Hyperlink sender, Microsoft.UI.Xaml.Documents.HyperlinkClickEventArgs args)
    {
        const string url = "https://github.com/landre-cerp/WCtrlDcsBiosBridge/tree/main/docs";
        try
        {
            Process.Start(new ProcessStartInfo(url) { UseShellExecute = true });
        }
        catch (Exception ex)
        {
            Logger.Warn(ex, "Failed to open docs URL");
        }
    }
}
