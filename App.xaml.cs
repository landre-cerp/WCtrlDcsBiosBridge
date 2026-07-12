using Microsoft.UI.Xaml;
using NLog;
using WCtrlDcsBiosBridge.Common;
using WCtrlDcsBiosBridge.Config;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge;

public partial class App : Application
{
    public static readonly Logger Logger = LogManager.GetCurrentClassLogger();

    private Window? _window;

    public App()
    {
        InitializeComponent();

        UnhandledException += (sender, args) =>
        {
            Logger.Error(args.Exception, "Unhandled exception (UI thread)");
            args.Handled = true;
        };

        AppDomain.CurrentDomain.UnhandledException += (sender, args) =>
        {
            Logger.Fatal(args.ExceptionObject as Exception, "Unhandled exception (background thread), IsTerminating={0}", args.IsTerminating);
            LogManager.Flush();
        };

        TaskScheduler.UnobservedTaskException += (sender, args) =>
        {
            Logger.Error(args.Exception, "Unobserved task exception");
            args.SetObserved();
        };
    }

    protected override void OnLaunched(LaunchActivatedEventArgs args)
    {
        // Apply saved theme before the main window renders. Language is handled by
        // Strings.CurrentLanguage + each control's Retranslate() (see Common/Strings.cs) —
        // ApplicationLanguages.PrimaryLanguageOverride doesn't work for this unpackaged app.
        var options = UserOptionsStorage.Load();
        ThemeManager.Apply(options.Theme);
        Strings.CurrentLanguage = options.Language;
        _window = new MainWindow();

        Logger.Info("Application started.");
        _window.Closed += (_, _) => Logger.Info("Application exited.");
        _window.Activate();
    }
}
