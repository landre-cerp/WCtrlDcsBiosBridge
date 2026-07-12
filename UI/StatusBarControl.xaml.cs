using Microsoft.UI;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;
using WCtrlDcsBiosBridge.Common;

namespace WCtrlDcsBiosBridge.UI;

public partial class StatusBarControl : UserControl
{
    private static readonly SolidColorBrush ErrorBrush = new(Colors.Red);
    private static readonly SolidColorBrush OkBrush = new(Colors.Green);

    public string Message { get; private set; } = Strings.Get("Status_Ready");
    public bool IsError { get; private set; }

    public StatusBarControl()
    {
        InitializeComponent();
        MessageText.Text = Message;
    }

    public void ShowStatus(string message, bool isError)
    {
        Message = message;
        IsError = isError;
        MessageText.Text = message;
        MessageText.Foreground = isError ? ErrorBrush : OkBrush;
    }

    /// <summary>Re-applies the static "Status:" label. The current message is left as-is —
    /// it reflects live state, not a fixed default (see MainWindow.Retranslate).</summary>
    public void Retranslate()
    {
        StatusLabel.Text = Strings.Get("StatusLabel");
    }
}
