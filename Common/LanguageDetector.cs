using Windows.System.UserProfile;
using WCtrlDcsBiosBridge.Config;

namespace WCtrlDcsBiosBridge.Common;

/// <summary>
/// Picks the best of our shipped translations (en/fr/de/es) for the current user, scanning
/// their full Windows preferred-language list rather than just the display language — so a
/// French user running an English-language Windows install still gets French, since French
/// shows up further down that list even though it isn't the display language.
/// </summary>
internal static class LanguageDetector
{
    public static readonly LanguagePreference[] SupportedLanguages =
    {
        LanguagePreference.English, LanguagePreference.French, LanguagePreference.German, LanguagePreference.Spanish
    };

    public static LanguagePreference DetectPreferredLanguage()
    {
        try
        {
            foreach (var tag in GlobalizationPreferences.Languages)
            {
                var match = MatchLanguage(tag);
                if (match != null) return match.Value;
            }
        }
        catch
        {
            // Fall through to the English default below.
        }
        return LanguagePreference.English;
    }

    private static LanguagePreference? MatchLanguage(string bcp47Tag)
    {
        var twoLetter = bcp47Tag.Length >= 2 ? bcp47Tag[..2].ToLowerInvariant() : bcp47Tag.ToLowerInvariant();
        return twoLetter switch
        {
            "fr" => LanguagePreference.French,
            "de" => LanguagePreference.German,
            "es" => LanguagePreference.Spanish,
            "en" => LanguagePreference.English,
            _ => null
        };
    }
}
