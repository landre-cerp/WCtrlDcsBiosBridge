namespace WCtrlDcsBiosBridge.Aircrafts.UH1H;

/// <summary>
/// Turns the raw ADF_FREQ output (the position of the tuning dial, 0-65535) into kHz.
/// The dial is not linear, and its scale is not the same on the three bands, so a straight
/// interpolation across a band is off by tens of kHz. Each band is a run of segments over which
/// the dial is linear. The segment boundaries were measured in the simulator and shared by Cory
/// (a community member) from his DCS-BIOS gauge definitions.
/// </summary>
internal static class UH1H_AdfScale
{
    /// <summary>One stretch of dial travel: raw positions <see cref="RawStart"/> to <see cref="RawEnd"/> read <see cref="KhzLow"/> to <see cref="KhzHigh"/>.</summary>
    private readonly record struct Segment(int RawStart, int RawEnd, int KhzLow, int KhzHigh);

    // ADF_BAND 0: 190-400 kHz.
    private static readonly Segment[] Low =
    [
        new(0, 2853, 190, 195), new(2854, 5509, 195, 200), new(5510, 8658, 200, 210),
        new(8659, 11708, 210, 220), new(11709, 14857, 220, 230), new(14858, 17710, 230, 240),
        new(17711, 20760, 240, 250), new(20761, 23712, 250, 260), new(23713, 26762, 260, 270),
        new(26763, 29615, 270, 280), new(29616, 32567, 280, 290), new(32568, 35519, 290, 300),
        new(35520, 38570, 300, 310), new(38571, 41423, 310, 320), new(41424, 44571, 320, 330),
        new(44572, 47523, 330, 340), new(47524, 50573, 340, 350), new(50574, 53426, 350, 360),
        new(53427, 56476, 360, 370), new(56477, 59428, 370, 380), new(59429, 62576, 380, 390),
        new(62577, 65535, 390, 400),
    ];

    // ADF_BAND 1: 400-850 kHz.
    //
    // 420-475 is refitted on a point checked in the simulator by ear: a 443 kHz beacon is
    // heard at raw 12222, where the measured 420-435-450-475 segments read 446. With 420
    // (6200) and 475 (16234) kept, that point fixes the dial at ~262 raw per kHz below 443 and
    // ~125 above it - close to the ~130 the table already had past 450 - so the two runs replace
    // the three segments without a kink. 408 kHz, also checked, is in the untouched 400-410.
    private static readonly Segment[] Mid =
    [
        new(0, 3050, 400, 410), new(3051, 6199, 410, 420),
        new(6200, 12222, 420, 443), new(12223, 16234, 443, 475), new(16235, 19481, 475, 500),
        new(19482, 22728, 500, 525), new(22729, 26270, 525, 550), new(26271, 29518, 550, 575),
        new(29519, 32666, 575, 600), new(32667, 35913, 600, 625), new(35914, 38963, 625, 650),
        new(38964, 42505, 650, 675), new(42506, 45850, 675, 700), new(45851, 49392, 700, 725),
        new(49393, 52442, 725, 750), new(52443, 55688, 750, 775), new(55689, 58837, 775, 800),
        new(58838, 62378, 800, 825), new(62379, 65535, 825, 850),
    ];

    // ADF_BAND 2: 850-1750 kHz (the dial runs on to 1800).
    private static readonly Segment[] High =
    [
        new(0, 2852, 850, 875), new(2853, 6197, 875, 900), new(6198, 9543, 900, 950),
        new(9544, 12987, 950, 1000), new(12988, 16332, 1000, 1050), new(16333, 19480, 1050, 1100),
        new(19481, 22825, 1100, 1150), new(22826, 26170, 1150, 1200), new(26171, 29417, 1200, 1250),
        new(29418, 32566, 1250, 1300), new(32567, 35911, 1300, 1350), new(35912, 39157, 1350, 1400),
        new(39158, 42404, 1400, 1450), new(42405, 45847, 1450, 1500), new(45848, 49192, 1500, 1550),
        new(49193, 52439, 1550, 1600), new(52440, 55685, 1600, 1650), new(55686, 59031, 1650, 1700),
        new(59032, 62376, 1700, 1750), new(62377, 65535, 1750, 1800),
    ];

    /// <summary>
    /// The frequency in kHz the dial reads at <paramref name="raw"/> on <paramref name="band"/>
    /// (ADF_BAND: 0 low, 1 mid, 2 high), whole kHz rounded down, as the dial scale is read.
    /// A raw value outside 0-65535 or an unknown band is clamped to the nearest end.
    /// </summary>
    public static int ToKhz(int band, int raw)
    {
        var segments = band switch
        {
            1 => Mid,
            2 => High,
            _ => Low,
        };

        raw = Math.Clamp(raw, 0, 65535);
        foreach (var segment in segments)
        {
            if (raw > segment.RawEnd) continue;

            var span = segment.RawEnd - segment.RawStart;
            var along = (raw - segment.RawStart) / (double)span;
            return (int)Math.Floor(segment.KhzLow + along * (segment.KhzHigh - segment.KhzLow));
        }

        return segments[^1].KhzHigh;
    }
}
