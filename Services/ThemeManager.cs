using System;
using System.Windows;
using Microsoft.Win32;
using WWCduDcsBiosBridge.Config;

namespace WWCduDcsBiosBridge.Services;

public static class ThemeManager
{
    private static ResourceDictionary? _activeDict;

    public static bool IsSystemDark()
    {
        try
        {
            var val = Registry.GetValue(
                @"HKEY_CURRENT_USER\Software\Microsoft\Windows\CurrentVersion\Themes\Personalize",
                "AppsUseLightTheme", 1);
            return val is int i && i == 0;
        }
        catch { return false; }
    }

    public static void Apply(ThemePreference preference)
    {
        var dark = preference switch
        {
            ThemePreference.Dark  => true,
            ThemePreference.Light => false,
            _                     => IsSystemDark()
        };

        var uri = dark
            ? new Uri("pack://application:,,,/Themes/Dark.xaml")
            : new Uri("pack://application:,,,/Themes/Light.xaml");

        var dicts = Application.Current.Resources.MergedDictionaries;
        if (_activeDict != null) dicts.Remove(_activeDict);
        _activeDict = new ResourceDictionary { Source = uri };
        dicts.Add(_activeDict);
    }
}
