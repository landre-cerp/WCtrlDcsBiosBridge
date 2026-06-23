using Newtonsoft.Json;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal record A10CExportData(
    [property: JsonProperty("cannon")]      int?               Cannon,
    [property: JsonProperty("stations")]    List<StationData>? Stations,
    [property: JsonProperty("engine")]      EngineData?        Engine,
    [property: JsonProperty("environment")] EnvironmentData?   Environment,
    [property: JsonProperty("position")]    PositionData?      Position);

internal record StationData(
    [property: JsonProperty("id")]    int     Id,
    [property: JsonProperty("count")] int     Count,
    [property: JsonProperty("name")]  string? Name);

internal record EngineData(
    [property: JsonProperty("rpm_left")]      double RpmLeft,
    [property: JsonProperty("rpm_right")]     double RpmRight,
    [property: JsonProperty("fuel_internal")] double FuelInternal,
    [property: JsonProperty("fuel_external")] double FuelExternal);

internal record EnvironmentData(
    [property: JsonProperty("wind_direction_deg")] int?    WindDirectionDeg,
    [property: JsonProperty("wind_speed_kts")]     double? WindSpeedKts);

internal record PositionData(
    [property: JsonProperty("lat")]    double? Lat,
    [property: JsonProperty("lon")]    double? Lon,
    [property: JsonProperty("alt_ft")] double? AltFt);
