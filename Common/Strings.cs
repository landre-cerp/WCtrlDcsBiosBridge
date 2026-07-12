using Microsoft.Windows.ApplicationModel.Resources;
using WCtrlDcsBiosBridge.Config;

namespace WCtrlDcsBiosBridge.Common;

/// <summary>
/// Looks up localized strings from Strings\&lt;lang&gt;\Resources.resw. XAML's automatic
/// x:Uid resolution only follows the OS's default resource context, which can't be
/// overridden for an unpackaged app (ApplicationLanguages.PrimaryLanguageOverride throws
/// here — see Services/LanguageManager history). So instead of relying on that, every
/// lookup here explicitly builds a ResourceContext pinned to <see cref="CurrentLanguage"/>,
/// and each window/control re-applies its own strings from code via Retranslate().
/// </summary>
internal static class Strings
{
    private static readonly ResourceManager Manager = new();
    private static readonly ResourceMap ResourceMap = Manager.MainResourceMap.GetSubtree("Resources");

    /// <summary>Set at startup and on every language toggle; drives which resw candidate Get/Format resolve.</summary>
    public static LanguagePreference CurrentLanguage { get; set; } = LanguagePreference.System;

    public static string Get(string key)
    {
        var context = BuildContext();
        var candidate = context is null ? ResourceMap.GetValue(key) : ResourceMap.GetValue(key, context);
        return candidate.ValueAsString;
    }

    public static string Format(string key, params object?[] args) => string.Format(Get(key), args);

    private static ResourceContext? BuildContext()
    {
        if (CurrentLanguage == LanguagePreference.System)
            return null;

        var context = Manager.CreateResourceContext();
        context.QualifierValues["Language"] = CurrentLanguage switch
        {
            LanguagePreference.French => "fr-FR",
            LanguagePreference.German => "de-DE",
            LanguagePreference.Spanish => "es-ES",
            _ => "en-US"
        };
        return context;
    }
}
