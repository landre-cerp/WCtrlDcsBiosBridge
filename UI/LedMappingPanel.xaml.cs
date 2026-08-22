using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using DCS_BIOS.ControlLocator;
using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using WCtrlDcsBiosBridge.Aircrafts;
using WCtrlDcsBiosBridge.Common;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Devices.Frontpanels;

namespace WCtrlDcsBiosBridge.UI;

/// <summary>
/// One row of the editor: a LED of the selected panel, and the binding (if any) the user
/// gave it for the selected aircraft.
/// </summary>
public sealed class LedRowViewModel : INotifyPropertyChanged
{
    private string _control = string.Empty;
    private LedCondition _condition = LedCondition.NotEquals;
    private double _value;
    private bool _needsCondition;
    private bool _isEnabled = true;
    private string _builtIn = "—";

    public LedRowViewModel(LedDescriptor led)
    {
        LedId = led.Id;
        Label = led.Label;
    }

    public string LedId { get; }
    public string Label { get; }

    /// <summary>The bound DCS-BIOS control identifier; empty when the LED is unbound.</summary>
    public string Control
    {
        get => _control;
        set => Set(ref _control, value ?? string.Empty);
    }

    public LedCondition Condition
    {
        get => _condition;
        set
        {
            if (Set(ref _condition, value))
                OnPropertyChanged(nameof(ConditionName));
        }
    }

    /// <summary>The threshold, as a double because that is what NumberBox binds to.</summary>
    public double Value
    {
        get => _value;
        set => Set(ref _value, value);
    }

    /// <summary>Whether the operator and threshold controls are shown for this row.</summary>
    public bool NeedsCondition
    {
        get => _needsCondition;
        set
        {
            if (Set(ref _needsCondition, value))
                OnPropertyChanged(nameof(ConditionVisibility));
        }
    }

    public Visibility ConditionVisibility => NeedsCondition ? Visibility.Visible : Visibility.Collapsed;

    /// <summary>
    /// What this LED does when the row is empty — the control the aircraft already drives it
    /// from, or a note that the listener works it out. Shown as the box's placeholder, so
    /// clearing a binding puts the built-in back in view as well as back in force.
    /// </summary>
    public string BuiltIn
    {
        get => _builtIn;
        set => Set(ref _builtIn, value);
    }

    /// <summary>False while the bridge is running this aircraft, so the row cannot be edited.</summary>
    public bool IsEnabled
    {
        get => _isEnabled;
        set => Set(ref _isEnabled, value);
    }

    public bool IsBound => !string.IsNullOrWhiteSpace(Control);

    public IReadOnlyList<string> ConditionNames => LedMappingPanel.ConditionLabels;

    /// <summary>The operator as its symbol, for the ComboBox.</summary>
    public string ConditionName
    {
        get => LedMappingPanel.LabelFor(Condition);
        set
        {
            if (LedMappingPanel.TryParseLabel(value, out var parsed))
                Condition = parsed;
        }
    }

    /// <summary>Re-reads the operator symbols after a language change.</summary>
    internal void RefreshConditionLabels()
    {
        OnPropertyChanged(nameof(ConditionNames));
        OnPropertyChanged(nameof(ConditionName));
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    private bool Set<T>(ref T field, T value, [CallerMemberName] string? name = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value)) return false;
        field = value;
        OnPropertyChanged(name);
        return true;
    }

    private void OnPropertyChanged([CallerMemberName] string? name = null) =>
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}

/// <summary>
/// Lets the user bind any DCS-BIOS indicator of an aircraft to any LED of a panel, so the
/// mapping is no longer whatever the code happens to hard-code. Writes ledmappings.json
/// through <see cref="LedMappingStore"/>; the bindings are picked up the next time a
/// listener starts, which is why the aircraft in use is read-only here.
/// </summary>
public sealed partial class LedMappingPanel : UserControl
{
    /// <summary>Raised when the user changed a binding and the file should be written.</summary>
    public event System.EventHandler? MappingChanged;

    /// <summary>The operators, in the order the ComboBox lists them.</summary>
    private static readonly LedCondition[] Conditions = System.Enum.GetValues<LedCondition>();

    /// <summary>
    /// The operator labels, parallel to <see cref="Conditions"/>. Symbols rather than the
    /// enum names: "≥" says what the row does where "AtLeast" had to be read, and each one
    /// takes the threshold box beside it.
    /// </summary>
    internal static string[] ConditionLabels { get; private set; } = BuildConditionLabels();

    private static string[] BuildConditionLabels() =>
        Conditions.Select(c => Strings.Get("LedMappingOp" + c)).ToArray();

    internal static string LabelFor(LedCondition condition) =>
        ConditionLabels[System.Array.IndexOf(Conditions, condition)];

    internal static bool TryParseLabel(string label, out LedCondition condition)
    {
        var index = System.Array.IndexOf(ConditionLabels, label);
        condition = index < 0 ? LedCondition.NotEquals : Conditions[index];
        return index >= 0;
    }

    /// <summary>
    /// The aircraft that can be configured: those DCS-BIOS actually exports controls for.
    /// The C-130J and F-14B(U) borrow another module's id purely so the control locator has
    /// something to load — binding their neighbour's controls would light nothing.
    /// </summary>
    private static IReadOnlyList<AircraftDescriptor> Configurable { get; } =
        AircraftRegistry.All.Where(d => d.DcsBiosModuleId is null).ToList();

    private readonly ObservableCollection<LedRowViewModel> _rows = new();

    /// <summary>Every binding the user has, across aircraft and panels — the file in memory.</summary>
    private List<LedBinding> _bindings = new();

    /// <summary>The selected aircraft's bindable controls, or empty when they could not be read.</summary>
    private IReadOnlyList<ControlChoice> _controls = System.Array.Empty<ControlChoice>();

    private string? _jsonDirectory;
    private string? _detectedAircraft;
    private bool _isLoading;

    public LedMappingPanel()
    {
        InitializeComponent();

        LedRows.ItemsSource = _rows;
        AircraftCombo.ItemsSource = Configurable.Select(d => d.DisplayName).ToList();
        DeviceCombo.ItemsSource = LedCatalog.Families.Select(f => FamilyLabel(f)).ToList();

        Reload();
    }

    /// <summary>
    /// The DCS-BIOS JSON directory from config.json. The editor reads module JSON directly
    /// rather than going through the aircraft the bridge has loaded, so it can be used while
    /// the bridge runs another module.
    /// </summary>
    public string? JsonDirectory
    {
        get => _jsonDirectory;
        set
        {
            _jsonDirectory = value;
            LoadControlsForSelectedAircraft();
        }
    }

    /// <summary>Rereads the mapping file into the editor.</summary>
    public void Reload()
    {
        _bindings = LedMappingStore.Current.Bindings.Select(Clone).ToList();

        if (AircraftCombo.SelectedIndex < 0 && Configurable.Count > 0) AircraftCombo.SelectedIndex = 0;
        if (DeviceCombo.SelectedIndex < 0 && LedCatalog.Families.Count > 0) DeviceCombo.SelectedIndex = 0;

        LoadControlsForSelectedAircraft();
    }

    /// <summary>The editor's current state as a mapping file, ready to save.</summary>
    public LedMappingFile BuildMapping() => new() { Bindings = _bindings.Select(Clone).ToList() };

    /// <summary>
    /// Marks the aircraft the bridge is currently running. Its bindings are registered when
    /// the listener starts, so editing them mid-flight would silently do nothing — the rows
    /// are locked and a badge says why, the same convention the options panel uses.
    /// </summary>
    public void SetDetectedAircraft(string? detectedAircraftName)
    {
        _detectedAircraft = detectedAircraftName;
        UpdateInUseState();
    }

    /// <summary>Re-applies every static string from Strings\&lt;lang&gt;\Resources.resw.</summary>
    public void Retranslate()
    {
        LedMappingHeader.Text = Strings.Get("LedMappingHeader");
        LedMappingIntro.Text = Strings.Get("LedMappingIntro");
        AircraftLabel.Text = Strings.Get("LedMappingAircraftLabel");
        DeviceLabel.Text = Strings.Get("LedMappingDeviceLabel");
        AdvancedOption.Content = Strings.Get("LedMappingAdvancedOption");
        LedColumnHeader.Text = Strings.Get("LedMappingLedHeader");
        ControlColumnHeader.Text = Strings.Get("LedMappingControlHeader");
        InUseBadgeText.Text = Strings.Get("LedMappingInUseBadge");
        RestartHint.Text = Strings.Get("LedMappingRestartHint");

        ConditionLabels = BuildConditionLabels();
        foreach (var row in _rows) row.RefreshConditionLabels();

        UpdateNotice();
    }

    // ── Selection ────────────────────────────────────────────────────────────

    private AircraftDescriptor? SelectedAircraft =>
        AircraftCombo.SelectedIndex >= 0 && AircraftCombo.SelectedIndex < Configurable.Count
            ? Configurable[AircraftCombo.SelectedIndex]
            : null;

    private LedDeviceFamily SelectedFamily =>
        DeviceCombo.SelectedIndex >= 0 && DeviceCombo.SelectedIndex < LedCatalog.Families.Count
            ? LedCatalog.Families[DeviceCombo.SelectedIndex]
            : LedDeviceFamily.FcuEfis;

    private void Aircraft_Changed(object sender, SelectionChangedEventArgs e) => LoadControlsForSelectedAircraft();

    private void Device_Changed(object sender, SelectionChangedEventArgs e) => RebuildRows();

    private void Advanced_Changed(object sender, RoutedEventArgs e) => UpdateNotice();

    // ── Control list ─────────────────────────────────────────────────────────

    /// <summary>
    /// Reads the selected module's controls straight from its JSON. Uses
    /// <c>onlyDirectResult</c> so the locator's shared control list — the one a running
    /// listener resolves its own controls against — is left untouched.
    /// </summary>
    private void LoadControlsForSelectedAircraft()
    {
        _controls = System.Array.Empty<ControlChoice>();

        var descriptor = SelectedAircraft;
        if (descriptor != null && !string.IsNullOrWhiteSpace(_jsonDirectory))
        {
            try
            {
                DCSBIOSControlLocator.JSONDirectory = _jsonDirectory;

                // onlyDirectResult: the locator's shared control list belongs to whatever
                // module the bridge is running — reading another one must not disturb it.
                _controls = DcsBiosControlCatalog.Bindable(
                    DCSBIOSControlLocator.GetModuleControlsFromJson(descriptor.JsonFile, onlyDirectResult: true));
            }
            catch (System.Exception ex)
            {
                App.Logger.Warn(ex, $"Could not read DCS-BIOS controls for {descriptor.DisplayName}");
            }
        }

        RebuildRows();
        UpdateNotice();
    }

    /// <summary>
    /// The controls offered in the picker: the module's indicator lamps, or every integer
    /// output when the user asked for all of them — or when the module declares no lamp at
    /// all, since an empty list would just look broken.
    /// </summary>
    private IReadOnlyList<ControlChoice> VisibleControls =>
        DcsBiosControlCatalog.Visible(_controls, showEverything: AdvancedOption.IsChecked == true);

    private void UpdateNotice()
    {
        var noJson = string.IsNullOrWhiteSpace(_jsonDirectory) || _controls.Count == 0;
        NoticeText.Text = noJson
            ? Strings.Get("LedMappingNoControls")
            : (AdvancedOption.IsChecked != true && !DcsBiosControlCatalog.HasLamps(_controls)
                ? Strings.Get("LedMappingNoLamps")
                : string.Empty);
        NoticeBorder.Visibility = string.IsNullOrEmpty(NoticeText.Text) ? Visibility.Collapsed : Visibility.Visible;
    }

    // ── Rows ─────────────────────────────────────────────────────────────────

    private void RebuildRows()
    {
        _isLoading = true;
        try
        {
            foreach (var row in _rows) row.PropertyChanged -= Row_PropertyChanged;
            _rows.Clear();

            var aircraft = SelectedAircraft?.DisplayName;
            var family = SelectedFamily;
            var defaults = aircraft is null
                ? AircraftLedDefaults.None
                : LedDefaults.ForDisplayName(aircraft);

            foreach (var led in LedCatalog.For(family))
            {
                var row = new LedRowViewModel(led) { BuiltIn = DescribeBuiltIn(led, family, defaults) };
                var existing = aircraft is null
                    ? null
                    : _bindings.FirstOrDefault(b =>
                        string.Equals(b.Aircraft, aircraft, System.StringComparison.OrdinalIgnoreCase)
                        && b.Device == family
                        && string.Equals(b.Led, led.Id, System.StringComparison.OrdinalIgnoreCase));

                if (existing != null)
                {
                    row.Control = existing.Control;
                    row.Condition = existing.Op;
                    row.Value = existing.Value;
                }

                row.NeedsCondition = FindControl(row.Control)?.NeedsCondition
                                     // Fallback for a control the catalog could not describe: show the
                                     // operator whenever the binding says something other than "≠ 0".
                                     ?? (row.Condition != LedCondition.NotEquals || row.Value != 0);
                row.PropertyChanged += Row_PropertyChanged;
                _rows.Add(row);
            }

            UpdateInUseState();
        }
        finally
        {
            _isLoading = false;
        }
    }

    /// <summary>
    /// What this aircraft already does with the LED, in the row's own words. The bare
    /// identifier rather than the picker's fuller wording: a placeholder sits behind an empty
    /// box and has to stay short, and the descriptions carry parentheses of their own.
    /// </summary>
    private string DescribeBuiltIn(LedDescriptor led, LedDeviceFamily family, AircraftLedDefaults defaults)
    {
        var info = family == LedDeviceFamily.Mcdu
            ? (LedCatalog.ParseMcduLed(led.Id) is McduLed mcduLed ? defaults.For(mcduLed) : LedDefaultInfo.None)
            : (led.Signal is FlightDeckSignal signal ? defaults.For(signal) : LedDefaultInfo.None);

        if (info.Describe() is { } controls)
            return Strings.Format("LedMappingBuiltInFormat", controls);

        if (info.ComputedFrom != null)
            return Strings.Format("LedMappingComputedFormat", info.ComputedFrom);

        return "—";
    }

    private ControlChoice? FindControl(string identifier) =>
        string.IsNullOrWhiteSpace(identifier)
            ? null
            : _controls.FirstOrDefault(c => string.Equals(c.Identifier, identifier, System.StringComparison.OrdinalIgnoreCase));

    private void Row_PropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (_isLoading || sender is not LedRowViewModel row) return;
        if (e.PropertyName is nameof(LedRowViewModel.IsEnabled)
            or nameof(LedRowViewModel.NeedsCondition)
            or nameof(LedRowViewModel.ConditionName)
            or nameof(LedRowViewModel.BuiltIn)) return;

        CommitRow(row);
    }

    /// <summary>Writes one row back into the in-memory file and asks for a save.</summary>
    private void CommitRow(LedRowViewModel row)
    {
        var aircraft = SelectedAircraft?.DisplayName;
        if (aircraft is null) return;

        var family = SelectedFamily;
        _bindings.RemoveAll(b =>
            string.Equals(b.Aircraft, aircraft, System.StringComparison.OrdinalIgnoreCase)
            && b.Device == family
            && string.Equals(b.Led, row.LedId, System.StringComparison.OrdinalIgnoreCase));

        if (row.IsBound)
        {
            _bindings.Add(new LedBinding
            {
                Aircraft = aircraft,
                Device = family,
                Led = row.LedId,
                Control = row.Control.Trim(),
                Op = row.Condition,
                Value = double.IsFinite(row.Value) && row.Value > 0 ? (uint)row.Value : 0,
            });
        }

        MappingChanged?.Invoke(this, System.EventArgs.Empty);
    }

    private void UpdateInUseState()
    {
        var inUse = _detectedAircraft != null
                    && string.Equals(SelectedAircraft?.DisplayName, _detectedAircraft, System.StringComparison.Ordinal);

        foreach (var row in _rows) row.IsEnabled = !inUse;
        InUseBadge.Visibility = inUse ? Visibility.Visible : Visibility.Collapsed;
    }

    // ── Row controls ─────────────────────────────────────────────────────────

    private void ControlBox_TextChanged(AutoSuggestBox sender, AutoSuggestBoxTextChangedEventArgs args)
    {
        if (args.Reason != AutoSuggestionBoxTextChangeReason.UserInput) return;

        var query = sender.Text?.Trim() ?? string.Empty;
        var matches = VisibleControls
            .Where(c => query.Length == 0
                        || c.Identifier.Contains(query, System.StringComparison.OrdinalIgnoreCase)
                        || c.Description.Contains(query, System.StringComparison.OrdinalIgnoreCase))
            .Take(50)
            .ToList();

        sender.ItemsSource = matches;

        // Typing something that matches no control unbinds the LED rather than silently
        // keeping the old one: what the box shows is what the LED follows.
        if (sender.DataContext is LedRowViewModel row && !matches.Any(m => m.Identifier == row.Control))
            SetRowControl(row, FindByDisplay(query));
    }

    private void ControlBox_SuggestionChosen(AutoSuggestBox sender, AutoSuggestBoxSuggestionChosenEventArgs args)
    {
        if (sender.DataContext is LedRowViewModel row && args.SelectedItem is ControlChoice choice)
            SetRowControl(row, choice);
    }

    private void ControlBox_QuerySubmitted(AutoSuggestBox sender, AutoSuggestBoxQuerySubmittedEventArgs args)
    {
        if (sender.DataContext is not LedRowViewModel row) return;

        var choice = args.ChosenSuggestion as ControlChoice ?? FindByDisplay(sender.Text);
        SetRowControl(row, choice);
        sender.Text = choice?.Display ?? string.Empty;
    }

    private ControlChoice? FindByDisplay(string? text)
    {
        if (string.IsNullOrWhiteSpace(text)) return null;
        return VisibleControls.FirstOrDefault(c =>
                   string.Equals(c.Display, text, System.StringComparison.OrdinalIgnoreCase))
               ?? VisibleControls.FirstOrDefault(c =>
                   string.Equals(c.Identifier, text.Trim(), System.StringComparison.OrdinalIgnoreCase));
    }

    private void SetRowControl(LedRowViewModel row, ControlChoice? choice)
    {
        row.NeedsCondition = choice?.NeedsCondition ?? false;
        if (!row.NeedsCondition) row.Condition = LedCondition.NotEquals;
        row.Control = choice?.Identifier ?? string.Empty;
    }

    private void Clear_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not FrameworkElement element || element.DataContext is not LedRowViewModel row) return;

        SetRowControl(row, null);
        row.Value = 0;

        // The box is this button's sibling. Its x:Name lives in the template's own namescope,
        // so looking it up from the item container would come back empty.
        if (element.Parent is Panel panel)
        {
            foreach (var child in panel.Children)
            {
                if (child is AutoSuggestBox box) box.Text = string.Empty;
            }
        }
    }

    private void ControlBox_Loaded(object sender, RoutedEventArgs e)
    {
        if (sender is AutoSuggestBox box && box.DataContext is LedRowViewModel row)
            box.Text = FindControl(row.Control)?.Display ?? row.Control;
    }

    private static LedBinding Clone(LedBinding binding) => new()
    {
        Aircraft = binding.Aircraft,
        Device = binding.Device,
        Led = binding.Led,
        Control = binding.Control,
        Op = binding.Op,
        Value = binding.Value,
    };

    private static string FamilyLabel(LedDeviceFamily family) => family switch
    {
        LedDeviceFamily.FcuEfis => "FCU / EFIS",
        LedDeviceFamily.Pap3 => "PAP-3",
        LedDeviceFamily.Agp32 => "AGP-32",
        LedDeviceFamily.Mcdu => "MCDU / PFP",
        _ => family.ToString(),
    };
}
