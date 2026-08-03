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
    [property: JsonProperty("cdnu")]        List<string>?    Cdnu,
    [property: JsonProperty("cni")]         CniData?         Cni);

internal record EnvironmentData(
    [property: JsonProperty("wind_direction_deg")] int?    WindDirectionDeg,
    [property: JsonProperty("wind_speed_kts")]     double? WindSpeedKts);

internal record PositionData(
    [property: JsonProperty("lat")]    double? Lat,
    [property: JsonProperty("lon")]    double? Lon,
    [property: JsonProperty("alt_ft")] double? AltFt);

/// <summary>
/// One C-130J CNI-MU page, scraped from the indicator's indication.
///
/// Only present when the page changed, plus a heartbeat every couple of seconds, so a null
/// here means "nothing new" and never "clear the screen".
///
/// The pilot's and the copilot's CNI both come this way, one per packet, named by
/// <see cref="Seat"/> — a page carries no marking of its own, so that name is the only thing
/// saying which screen it belongs on.
///
/// The indication carries names and values and nothing else: no position, no font size, no
/// inverted state. Laying the page out therefore needs the offline schema in
/// <c>tools/cni-schema</c>, matched against these blocks.
/// </summary>
internal record CniData(
    [property: JsonProperty("seat")]   string?          Seat,
    [property: JsonProperty("idx")]    int?             Idx,
    [property: JsonProperty("title")]  string?          Title,
    [property: JsonProperty("n")]      int?             N,
    [property: JsonProperty("blocks")] List<CniBlock>?  Blocks,
    [property: JsonProperty("radios")] List<RadioState>? Radios,

    /// <summary>
    /// The INAV the aircraft has settled on, read off the PFD rather than the CNI. Every page
    /// draws its own INAV number in the top-left corner and boxes it when the two agree, which
    /// is why the pilot's 1 and the copilot's 2 are never both boxed.
    /// </summary>
    [property: JsonProperty("soln")] int? ShipSolution);

/// <summary>
/// One radio, read from its device rather than from the display.
///
/// The page shows a radio's power as OFF/ON with one word highlighted, and the indication
/// carries neither the highlight nor anything that moves when it changes — this is the only
/// place that state can be had. <see cref="Frequency"/> is what says which radio a page is
/// talking about: the page prints what it is tuned to, so no device number is needed at either
/// end. In kilohertz, because the raw reading drifts by a few hundred hertz between frames.
/// </summary>
internal record RadioState(
    [property: JsonProperty("i")]  int?  Device,
    [property: JsonProperty("f")]  long? Frequency,
    [property: JsonProperty("on")] bool? On);

/// <summary>
/// One element of a CNI page. <see cref="N"/> counts every node in document order, children
/// included, so it is unique within a packet.
///
/// <see cref="K"/> is the element name. Two are stable and meaningful — <c>cni_title</c> and
/// <c>cni_scratchpad</c> — and the rest are GUIDs the sim regenerates every session, useful
/// for spotting that a field changed within a session but never as a lookup key.
///
/// <see cref="C"/> is set on the container a toggle draws around its visible words. Which
/// container was emitted decides which word is highlighted in the cockpit, but the two are
/// indistinguishable from here: same ordinal, same children, same values.
/// </summary>
internal record CniBlock(
    [property: JsonProperty("n")] int?             N,
    [property: JsonProperty("k")] string?          K,
    [property: JsonProperty("v")] string?          V,
    [property: JsonProperty("c")] List<CniBlock>?  C);
