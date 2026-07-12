using Microsoft.UI;
using Microsoft.UI.Xaml.Controls;
using Microsoft.UI.Xaml.Media;

namespace WCtrlDcsBiosBridge.UI;

public partial class StatusBarControl : UserControl
{
    private static readonly SolidColorBrush ErrorBrush = new(Colors.Red);
    private static readonly SolidColorBrush OkBrush = new(Colors.Green);

    public string Message { get; private set; } = "Ready.";
    public bool IsError { get; private set; }

    public StatusBarControl()
    {
        InitializeComponent();
    }

    public void ShowStatus(string message, bool isError)
    {
        Message = message;
        IsError = isError;
        MessageText.Text = message;
        MessageText.Foreground = isError ? ErrorBrush : OkBrush;
    }
}
