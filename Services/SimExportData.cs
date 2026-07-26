using Newtonsoft.Json;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// One packet from wctrl-export.lua. Everything below the envelope is optional:
/// availability depends on the module and on what the mission allows exporting,
/// so consumers must degrade rather than assume a field is present.
/// </summary>
internal record SimExportData(
    [property: JsonProperty("ver")]         int?             Ver,
    [property: JsonProperty("aircraft")]    string?          Aircraft,
    [property: JsonProperty("environment")] EnvironmentData? Environment,
    [property: JsonProperty("position")]    PositionData?    Position,
    [property: JsonProperty("cdnu")]        List<string>?    Cdnu);

internal record EnvironmentData(
    [property: JsonProperty("wind_direction_deg")] int?    WindDirectionDeg,
    [property: JsonProperty("wind_speed_kts")]     double? WindSpeedKts);

internal record PositionData(
    [property: JsonProperty("lat")]    double? Lat,
    [property: JsonProperty("lon")]    double? Lon,
    [property: JsonProperty("alt_ft")] double? AltFt);
