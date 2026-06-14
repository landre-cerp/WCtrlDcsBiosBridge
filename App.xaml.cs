using NLog;
using System.Windows;
using WWCduDcsBiosBridge.Config;
using WWCduDcsBiosBridge.Services;

namespace WWCduDcsBiosBridge;

public partial class App : Application
{
    public static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    protected override void OnStartup(StartupEventArgs e)
    {
        this.DispatcherUnhandledException += (sender, args) =>
        {
            MessageBox.Show($"An unexpected error occurred: {args.Exception.Message}",
                            "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            args.Handled = true;
        };

        base.OnStartup(e);

        // Apply saved theme before the main window renders
        var options = UserOptionsStorage.Load();
        ThemeManager.Apply(options.Theme);

        Logger.Info("Application started.");
    }

    protected override void OnExit(ExitEventArgs e)
    {
        Logger.Info("Application exited.");
        base.OnExit(e);
    }
}
