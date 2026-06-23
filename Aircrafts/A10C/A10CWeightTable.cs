namespace WCtrlDcsBiosBridge.Aircrafts;

internal static class A10CWeightTable
{
    public const int EmptyWeightLbs     = 25629;
    public const int MaxInternalFuelLbs = 11087;
    private const double LbsPerShell   = 1775.0 / 1150.0; // based on max 1150 rounds = 1775 lbs

    // Key: (DCS weapon name from LoGetNameByType, count on that station)
    // Value: total pylon weight lbs (rack + munitions), ported from A10C-calc/src/data/A10C.ts
    private static readonly Dictionary<(string Name, int Count), int> _table = new()
    {
        // ECM / Pods
        [("ALQ-184",    1)] = 474,
        [("ALQ-131",    1)] = 672,
        [("AN/AAQ-28",  1)] = 459,   // contains-matched: "AN/AAQ-28 LITENING"

        // AAMs
        [("AIM-9L",  1)] = 256,
        [("AIM-9L",  2)] = 443,
        [("AIM-9M",  1)] = 256,
        [("AIM-9M",  2)] = 443,

        // AGMs — Maverick (single LAU-117 vs LAU-88 rack)
        [("AGM-65D", 1)] = 611,
        [("AGM-65D", 2)] = 1426,
        [("AGM-65D", 3)] = 1907,
        [("AGM-65G", 1)] = 794,
        [("AGM-65H", 1)] = 589,
        [("AGM-65H", 2)] = 1382,
        [("AGM-65H", 3)] = 1841,
        [("AGM-65K", 1)] = 785,
        [("AGM-65L", 1)] = 774,

        // LGBs / JDAMs
        [("GBU-10",       1)] = 2114,
        [("GBU-12",       1)] = 611,
        [("GBU-12",       3)] = 1956,   // BRU-42 TER
        [("GBU-31",       1)] = 2059,
        [("GBU-38",       1)] = 531,
        [("GBU-54",       1)] = 558,

        // GP Bombs
        [("MK-82",        1)] = 503,
        [("MK-82 AIR",    1)] = 534,
        [("MK-82",        3)] = 1631,   // BRU-42 TER
        [("MK-82 AIR",    3)] = 1724,
        [("MK-84",        1)] = 2000,

        // CBUs
        [("CBU-87",  1)] = 948,
        [("CBU-97",  1)] = 919,
        [("CBU-103", 1)] = 948,
        [("CBU-105", 1)] = 919,

        // APKWS (laser-guided Hydra)
        [("APKWS",  7)] = 298,    // single LAU-131 × 7
        [("APKWS", 14)] = 674,    // 2 × LAU-131 (approximate; not in TS table)
        [("APKWS", 21)] = 1001,   // BRU-42 + 3 × LAU-131

        // Hydra unguided (DCS name TBD — verify with test_client.py)
        [("M151",  7)] = 225,
        [("M151", 21)] = 897,
        [("M156",  7)] = 229,
        [("M156", 21)] = 904,
        [("M257",  7)] = 238,
        [("M257", 21)] = 933,
        [("MK5",   7)] = 201,
        [("MK5",  21)] = 822,
    };

    /// <summary>
    /// Returns pylon weight in lbs for a DCS weapon name and count, or null when unknown.
    /// Uses a contains-based match to handle DCS name suffixes (e.g. "AN/AAQ-28 LITENING").
    /// </summary>
    public static int? StationWeight(string dcsName, int count)
    {
        if (_table.TryGetValue((dcsName, count), out var exact)) return exact;

        foreach (var kv in _table)
            if (kv.Key.Count == count &&
                dcsName.Contains(kv.Key.Name, StringComparison.OrdinalIgnoreCase))
                return kv.Value;

        return null;
    }

    public static int CannonWeight(int shells) =>
        (int)Math.Round(shells * LbsPerShell);

    public static int FuelWeight(double fraction) =>
        (int)Math.Round(fraction * MaxInternalFuelLbs);

    /// <summary>
    /// Computes gross weight from live export data.
    /// Returns (gw lbs, hasUnknown) where hasUnknown means at least one station
    /// had no weight table entry — GW is a lower bound in that case.
    /// </summary>
    public static (int Gw, bool HasUnknown) ComputeGw(A10CExportData data)
    {
        int gw = EmptyWeightLbs;
        bool hasUnknown = false;

        if (data.Engine != null)
            gw += FuelWeight(data.Engine.FuelInternal);

        if (data.Cannon.HasValue)
            gw += CannonWeight(data.Cannon.Value);

        if (data.Stations != null)
        {
            foreach (var st in data.Stations)
            {
                if (st.Count == 0 || st.Name == null) continue;
                var w = StationWeight(st.Name, st.Count);
                if (w.HasValue) gw += w.Value;
                else            hasUnknown = true;
            }
        }

        return (gw, hasUnknown);
    }
}
