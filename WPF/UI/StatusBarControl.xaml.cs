using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Controls;

namespace WCtrlDcsBiosBridge.UI;

public partial class StatusBarControl : UserControl, INotifyPropertyChanged
{
    private string _message = "Ready.";
    private bool _isError;

    public string Message
    {
        get => _message;
        set
        {
            if (_message != value)
            {
                _message = value;
                OnPropertyChanged();
            }
        }
    }

    public bool IsError
    {
        get => _isError;
        set
        {
            if (_isError != value)
            {
                _isError = value;
                OnPropertyChanged();
            }
        }
    }

    public StatusBarControl()
    {
        InitializeComponent();
        DataContext = this;
    }

    public void ShowStatus(string message, bool isError)
    {
        Message = message;
        IsError = isError;
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    protected virtual void OnPropertyChanged([CallerMemberName] string? propertyName = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
