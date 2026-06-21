using DCS_BIOS;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// Forwards selected CDU keys (LSKs + a subset of typing keys like digits and '/') to the sim
/// over DCS-BIOS while the live (default) page is shown. The physical MCDU keys are also bound
/// inside DCS, so when a custom page (e.g. the takeoff-perf page) uses them for its own input
/// they would otherwise also drive the real aircraft CDU. The intended workflow: the user
/// unbinds these keys in DCS, and the bridge drives the real A-10C CDU here — but only on the
/// default page. On a custom page the keys are consumed locally and nothing is sent to the sim.
/// </summary>
internal partial class A10C_Listener
{
    // Physical MCDU keys -> A-10C CDU button identifiers. Every key a custom page consumes
    // (so the user unbinds it in DCS) needs an entry here so the real CDU still works on the
    // default page: the four LSKs per side (display rows 3/5/7/9), the digits and the slash
    // (used by the wind entry). The top four physical LSKs are assumed to map to 3/5/7/9 —
    // adjust if your device's A-10C overlay differs.
    private static readonly IReadOnlyDictionary<Key, string> _cduKeyToSim = new Dictionary<Key, string>
    {
        [Key.LineSelectLeft1] = "CDU_LSK_3L",
        [Key.LineSelectLeft2] = "CDU_LSK_5L",
        [Key.LineSelectLeft3] = "CDU_LSK_7L",
        [Key.LineSelectLeft4] = "CDU_LSK_9L",
        [Key.LineSelectRight1] = "CDU_LSK_3R",
        [Key.LineSelectRight2] = "CDU_LSK_5R",
        [Key.LineSelectRight3] = "CDU_LSK_7R",
        [Key.LineSelectRight4] = "CDU_LSK_9R",

        [Key.Digit0] = "CDU_0",
        [Key.Digit1] = "CDU_1",
        [Key.Digit2] = "CDU_2",
        [Key.Digit3] = "CDU_3",
        [Key.Digit4] = "CDU_4",
        [Key.Digit5] = "CDU_5",
        [Key.Digit6] = "CDU_6",
        [Key.Digit7] = "CDU_7",
        [Key.Digit8] = "CDU_8",
        [Key.Digit9] = "CDU_9",
        [Key.Slash] = "CDU_SLASH",
    };

    /// <summary>
    /// Press + release the mapped A-10C CDU button over DCS-BIOS. Call only on the default
    /// page; unmapped keys are ignored.
    /// </summary>
    private static void ForwardCduKeyToSim(Key key)
    {
        if (!_cduKeyToSim.TryGetValue(key, out var id)) return;
        DCSBIOS.Send("A10C_TKOFF", $"{id} 1\n");
        DCSBIOS.Send("A10C_TKOFF", $"{id} 0\n");
    }
}
