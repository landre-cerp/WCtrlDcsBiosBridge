using System.Globalization;
using System.Windows.Data;

namespace WCtrlDcsBiosBridge.UI.Converters;

/// <summary>
/// Drives an aircraft options section's enabled state. Given the section's own
/// name and the currently detected aircraft name, returns <c>false</c> (disabled)
/// only when they match — so a section is greyed out exactly while its aircraft is
/// running. Bind: [0] = section name, [1] = detected aircraft name.
/// </summary>
public sealed class AircraftNotActiveConverter : IMultiValueConverter
{
    public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
    {
        var sectionName = values.Length > 0 ? values[0] as string : null;
        var detectedName = values.Length > 1 ? values[1] as string : null;
        return !string.Equals(sectionName, detectedName, StringComparison.Ordinal);
    }

    public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        => throw new NotSupportedException();
}
