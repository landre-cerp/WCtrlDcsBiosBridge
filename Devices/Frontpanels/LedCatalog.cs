using System;
using System.Collections.Generic;
using System.Linq;
using WCtrlDcsBiosBridge.Aircrafts;
using WwDevicesDotNet;
using WwDevicesDotNet.Winctrl.Agp32;
using WwDevicesDotNet.Winctrl.FcuAndEfis;
using WwDevicesDotNet.Winctrl.Pap3;

namespace WCtrlDcsBiosBridge.Devices.Frontpanels;

/// <summary>
/// The LED-carrying device families a user binding can target. PDC-3 is absent
/// on purpose: it drives brightness only and has no LEDs of its own.
/// </summary>
public enum LedDeviceFamily
{
    FcuEfis,
    Pap3,
    Agp32,
    Mcdu,
}

/// <summary>The six MCDU annunciators, as named on the CDU state.</summary>
public enum McduLed
{
    Fail,
    Fm1,
    Fm2,
    Fm,
    Ind,
    Rdy,
}

/// <summary>
/// One bindable LED on a device family: a stable id for the mapping file, the label
/// silkscreened on the hardware, and — for frontpanel families — the setter that writes
/// it into that family's LED object. <see cref="Set"/> is null for
/// <see cref="LedDeviceFamily.Mcdu"/>, whose LEDs live on the CDU state rather than an
/// <see cref="IFrontpanelLeds"/>; use <see cref="LedCatalog.ParseMcduLed"/> there.
/// </summary>
/// <remarks>
/// Labels are not translated: they are the legends printed on the panels, which read the
/// same in every language.
/// </remarks>
/// <param name="Signal">
/// The semantic signal this LED shows when the user has not bound it. Null for a LED no
/// aircraft drives on its own — the FCU/EFIS and PAP3 legends, which have no equivalent in
/// the flight deck model and stay dark until somebody binds them.
/// </param>
public sealed record LedDescriptor(
    string Id,
    string Label,
    Action<IFrontpanelLeds, bool>? Set = null,
    FlightDeckSignal? Signal = null);

/// <summary>
/// What is bindable on each device family. Written out by hand rather than reflected over
/// the LED classes because the families do not agree on shape — FCU/EFIS and PAP3 expose
/// bool properties, the AGP32 an enum plus a Set method — and because explicit entries give
/// stable ids that survive a rename in the device library.
/// </summary>
public static class LedCatalog
{
    private static LedDescriptor Fcu(string id, string label, Action<FcuEfisLeds, bool> set) =>
        new(id, label, (leds, on) => { if (leds is FcuEfisLeds l) set(l, on); });

    private static LedDescriptor Pap(string id, string label, Action<Pap3Leds, bool> set) =>
        new(id, label, (leds, on) => { if (leds is Pap3Leds l) set(l, on); });

    private static LedDescriptor Agp(string id, string label, Agp32State.Agp32Led led, FlightDeckSignal signal) =>
        new(id, label, (leds, on) => { if (leds is Agp32State.Agp32Leds l) l.Set(led, on); }, signal);

    private static readonly IReadOnlyList<LedDescriptor> _fcuEfis = new[]
    {
        Fcu("Loc",       "LOC",       (l, v) => l.Loc = v),
        Fcu("Ap1",       "AP1",       (l, v) => l.Ap1 = v),
        Fcu("Ap2",       "AP2",       (l, v) => l.Ap2 = v),
        Fcu("AThr",      "A/THR",     (l, v) => l.AThr = v),
        Fcu("Exped",     "EXPED",     (l, v) => l.Exped = v),
        Fcu("Appr",      "APPR",      (l, v) => l.Appr = v),
        Fcu("LeftFd",    "FD (L)",    (l, v) => l.LeftFd = v),
        Fcu("LeftLs",    "LS (L)",    (l, v) => l.LeftLs = v),
        Fcu("LeftCstr",  "CSTR (L)",  (l, v) => l.LeftCstr = v),
        Fcu("LeftWpt",   "WPT (L)",   (l, v) => l.LeftWpt = v),
        Fcu("LeftVorD",  "VOR.D (L)", (l, v) => l.LeftVorD = v),
        Fcu("LeftNdb",   "NDB (L)",   (l, v) => l.LeftNdb = v),
        Fcu("LeftArpt",  "ARPT (L)",  (l, v) => l.LeftArpt = v),
        Fcu("RightFd",   "FD (R)",    (l, v) => l.RightFd = v),
        Fcu("RightLs",   "LS (R)",    (l, v) => l.RightLs = v),
        Fcu("RightCstr", "CSTR (R)",  (l, v) => l.RightCstr = v),
        Fcu("RightWpt",  "WPT (R)",   (l, v) => l.RightWpt = v),
        Fcu("RightVorD", "VOR.D (R)", (l, v) => l.RightVorD = v),
        Fcu("RightNdb",  "NDB (R)",   (l, v) => l.RightNdb = v),
        Fcu("RightArpt", "ARPT (R)",  (l, v) => l.RightArpt = v),
    };

    private static readonly IReadOnlyList<LedDescriptor> _pap3 = new[]
    {
        Pap("N1",      "N1",       (l, v) => l.N1 = v),
        Pap("Speed",   "SPEED",    (l, v) => l.Speed = v),
        Pap("Vnav",    "VNAV",     (l, v) => l.Vnav = v),
        Pap("LvlChg",  "LVL CHG",  (l, v) => l.LvlChg = v),
        Pap("HdgSel",  "HDG SEL",  (l, v) => l.HdgSel = v),
        Pap("Lnav",    "LNAV",     (l, v) => l.Lnav = v),
        Pap("VorLoc",  "VOR LOC",  (l, v) => l.VorLoc = v),
        Pap("App",     "APP",      (l, v) => l.App = v),
        Pap("AltHold", "ALT HOLD", (l, v) => l.AltHold = v),
        Pap("Vs",      "V/S",      (l, v) => l.Vs = v),
        Pap("CmdA",    "CMD A",    (l, v) => l.CmdA = v),
        Pap("CwsA",    "CWS A",    (l, v) => l.CwsA = v),
        Pap("CmdB",    "CMD B",    (l, v) => l.CmdB = v),
        Pap("CwsB",    "CWS B",    (l, v) => l.CwsB = v),
        Pap("AtArm",   "A/T ARM",  (l, v) => l.AtArm = v),
        Pap("FdL",     "FD (L)",   (l, v) => l.FdL = v),
        Pap("FdR",     "FD (R)",   (l, v) => l.FdR = v),
    };

    private static readonly IReadOnlyList<LedDescriptor> _agp32 = new[]
    {
        Agp("Gear1Unlk",       "GEAR 1 UNLK",        Agp32State.Agp32Led.Gear1Unlk, FlightDeckSignal.GearWarning),
        Agp("Gear2Unlk",       "GEAR 2 UNLK",        Agp32State.Agp32Led.Gear2Unlk, FlightDeckSignal.GearWarning),
        Agp("Gear3Unlk",       "GEAR 3 UNLK",        Agp32State.Agp32Led.Gear3Unlk, FlightDeckSignal.GearWarning),
        Agp("Gear1Down",       "GEAR 1 DOWN",        Agp32State.Agp32Led.Gear1Down, FlightDeckSignal.GearLeftDown),
        Agp("Gear2Down",       "GEAR 2 DOWN",        Agp32State.Agp32Led.Gear2Down, FlightDeckSignal.GearNoseDown),
        Agp("Gear3Down",       "GEAR 3 DOWN",        Agp32State.Agp32Led.Gear3Down, FlightDeckSignal.GearRightDown),
        Agp("GearDownRed",     "GEAR DOWN (red)",    Agp32State.Agp32Led.GearDownRed, FlightDeckSignal.GearWarning),
        Agp("BrkFanHot",       "BRK FAN HOT",        Agp32State.Agp32Led.BrkFanHot, FlightDeckSignal.BrkFanHot),
        Agp("BrkFanOn",        "BRK FAN ON",         Agp32State.Agp32Led.BrkFanOn, FlightDeckSignal.BrkFanOn),
        Agp("AutoBrkLoDecel",  "AUTO BRK LO DECEL",  Agp32State.Agp32Led.AutoBrkLoDecel, FlightDeckSignal.AutoBrkLoDecel),
        Agp("AutoBrkMedDecel", "AUTO BRK MED DECEL", Agp32State.Agp32Led.AutoBrkMedDecel, FlightDeckSignal.AutoBrkMedDecel),
        Agp("AutoBrkMaxDecel", "AUTO BRK MAX DECEL", Agp32State.Agp32Led.AutoBrkMaxDecel, FlightDeckSignal.AutoBrkMaxDecel),
        Agp("AutoBrkLoOn",     "AUTO BRK LO ON",     Agp32State.Agp32Led.AutoBrkLoOn, FlightDeckSignal.AutoBrkLoOn),
        Agp("AutoBrkMedOn",    "AUTO BRK MED ON",    Agp32State.Agp32Led.AutoBrkMedOn, FlightDeckSignal.AutoBrkMedOn),
        Agp("AutoBrkMaxOn",    "AUTO BRK MAX ON",    Agp32State.Agp32Led.AutoBrkMaxOn, FlightDeckSignal.AutoBrkMaxOn),
        Agp("TerrOnNdOn",      "TERR ON ND",         Agp32State.Agp32Led.TerrOnNdOn, FlightDeckSignal.TerrOnNd),
    };

    private static readonly IReadOnlyList<LedDescriptor> _mcdu = new[]
    {
        new LedDescriptor("Fail", "FAIL"),
        new LedDescriptor("Fm1",  "FM1"),
        new LedDescriptor("Fm2",  "FM2"),
        new LedDescriptor("Fm",   "FM"),
        new LedDescriptor("Ind",  "IND"),
        new LedDescriptor("Rdy",  "RDY"),
    };

    /// <summary>The LEDs a user can bind on <paramref name="family"/>, in panel order.</summary>
    public static IReadOnlyList<LedDescriptor> For(LedDeviceFamily family) => family switch
    {
        LedDeviceFamily.FcuEfis => _fcuEfis,
        LedDeviceFamily.Pap3 => _pap3,
        LedDeviceFamily.Agp32 => _agp32,
        LedDeviceFamily.Mcdu => _mcdu,
        _ => Array.Empty<LedDescriptor>(),
    };

    /// <summary>Every family that has bindable LEDs.</summary>
    public static IReadOnlyList<LedDeviceFamily> Families { get; } =
        Enum.GetValues<LedDeviceFamily>().Where(f => For(f).Count > 0).ToArray();

    /// <summary>Finds a LED by its mapping-file id, or null when the id is unknown.</summary>
    public static LedDescriptor? Find(LedDeviceFamily family, string ledId) =>
        For(family).FirstOrDefault(d => string.Equals(d.Id, ledId, StringComparison.OrdinalIgnoreCase));

    /// <summary>
    /// Resolves an MCDU LED id to the enum the listener writes. Returns null for an
    /// unknown id, which the caller reports rather than throwing.
    /// </summary>
    public static McduLed? ParseMcduLed(string ledId) =>
        Enum.TryParse<McduLed>(ledId, ignoreCase: true, out var led) ? led : null;
}
