using Microsoft.UI.Xaml;
using Microsoft.UI.Xaml.Controls;
using NLog;
using WCtrlDcsBiosBridge.Common;
using WCtrlDcsBiosBridge.Services;
using Windows.ApplicationModel.DataTransfer;

namespace WCtrlDcsBiosBridge.UI;

/// <summary>
/// In-window overlay panel showing the tail of the application log, so a user hitting a
/// failure can read the exception and paste it into a bug report without being told where
/// log.txt lives. Hosted inside MainWindow rather than opened as its own window — see
/// ConfigPanel.xaml.cs for why secondary windows are avoided here.
/// </summary>
public sealed partial class LogViewerPanel : UserControl
{
    private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    /// <summary>What Copy puts on the clipboard: the log as shown, or empty when the panel
    /// is displaying a message about why there is nothing to show.</summary>
    private string _text = string.Empty;

    /// <summary>The unfiltered tail, kept so toggling the filter off does not have to re-read
    /// the file — and so it cannot come back different from what the user was just reading.</summary>
    private string _tail = string.Empty;

    private TaskCompletionSource? _tcs;

    public LogViewerPanel()
    {
        InitializeComponent();
        Retranslate();
    }

    /// <summary>Re-applies every static string from Strings\&lt;lang&gt;\Resources.resw.
    /// The body and the path are live content, not fixed defaults, and are left alone.</summary>
    public void Retranslate()
    {
        LogViewerHeader.Text = Strings.Get("LogViewerHeader");
        ErrorsOnlyToggle.Content = Strings.Get("LogViewerErrorsOnlyButtonControl");
        CopyButton.Content = Strings.Get("LogViewerCopyButtonControl");
        CloseButton.Content = Strings.Get("LogViewerCloseButtonControl");
    }

    /// <summary>
    /// Loads the tail and returns a task that completes when the user closes the panel, so
    /// the caller can await it and collapse the overlay on a single path.
    /// </summary>
    public Task ShowAsync()
    {
        StatusText.Visibility = Visibility.Collapsed;
        LoadLog();

        _tcs = new TaskCompletionSource();
        return _tcs.Task;
    }

    private void LoadLog()
    {
        _tail = string.Empty;

        var path = LogFileAccess.ResolvePath();
        LogPathText.Text = path ?? string.Empty;

        if (path == null || !File.Exists(path))
        {
            SetBody(string.Empty, Strings.Get("LogViewer_NotFound"));
            return;
        }

        try
        {
            _tail = LogFileAccess.ReadTail(path);
            ShowTail();
        }
        catch (Exception ex)
        {
            Logger.Warn(ex, "Failed to read the log file for the viewer");
            SetBody(string.Empty, Strings.Format("LogViewer_ReadFailedFormat", ex.Message));
        }
    }

    /// <summary>
    /// Puts the tail on screen through the filter as it currently stands. Called both on load
    /// and on every toggle, so the two paths cannot drift apart.
    /// </summary>
    private void ShowTail()
    {
        var errorsOnly = ErrorsOnlyToggle.IsChecked == true;
        var body = errorsOnly ? LogFileAccess.FilterErrors(_tail) : _tail;

        if (body.Length > 0)
        {
            SetBody(body, body);
            return;
        }

        // Nothing to show, and which of the two reasons it is decides what the user does next:
        // an empty log means the app has not logged anything, no errors means it has and none
        // of it went wrong. Telling them "empty" in the second case would be a lie.
        SetBody(string.Empty,
                Strings.Get(errorsOnly && _tail.Length > 0 ? "LogViewer_NoErrors" : "LogViewer_Empty"));
    }

    private void ErrorsOnlyToggle_Toggled(object sender, RoutedEventArgs e)
    {
        // The copy confirmation refers to what was on screen a moment ago; the filter has just
        // changed what that is.
        StatusText.Visibility = Visibility.Collapsed;
        ShowTail();
    }

    /// <summary>
    /// Sets what is displayed and, separately, what Copy will yield — they diverge whenever
    /// the panel is showing an explanation rather than log content, and copying "No log file
    /// found." into a bug report would help nobody.
    /// </summary>
    private void SetBody(string copyText, string displayText)
    {
        _text = copyText;
        LogText.Text = displayText;

        // The interesting lines are the most recent ones, at the bottom.
        LogScroll.UpdateLayout();
        LogScroll.ChangeView(null, LogScroll.ScrollableHeight, null, disableAnimation: true);
    }

    private void CopyButton_Click(object sender, RoutedEventArgs e)
    {
        if (_text.Length == 0) return;

        try
        {
            var package = new DataPackage();
            package.SetText(_text);
            Clipboard.SetContent(package);

            StatusText.Text = Strings.Get("LogViewer_Copied");
            StatusText.Visibility = Visibility.Visible;
        }
        catch (Exception ex)
        {
            Logger.Warn(ex, "Failed to copy the log to the clipboard");
        }
    }

    private void CloseButton_Click(object sender, RoutedEventArgs e)
    {
        // Dropped rather than kept: a viewer left holding a few hundred KB of log for the
        // life of the process is the one thing this panel should not do.
        _text = string.Empty;
        _tail = string.Empty;
        LogText.Text = string.Empty;

        _tcs?.TrySetResult();
        _tcs = null;
    }
}
