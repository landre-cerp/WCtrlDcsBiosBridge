using WCtrlDcsBiosBridge.Aircrafts.UH1H;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

public class UH1HAdfScaleTests
{
    [Theory]
    [InlineData(0, 0, 190)]
    [InlineData(0, 65535, 400)]
    [InlineData(1, 0, 400)]
    [InlineData(1, 65535, 850)]
    [InlineData(2, 0, 850)]
    [InlineData(2, 65535, 1800)]
    public void BandEndsReadTheirNominalFrequency(int band, int raw, int expectedKhz) =>
        Assert.Equal(expectedKhz, UH1H_AdfScale.ToKhz(band, raw));

    [Theory]
    [InlineData(0, 5510, 200)]       // start of a segment
    [InlineData(0, 8658, 210)]       // end of that segment
    [InlineData(1, 16235, 475)]      // the segments widen from 10 to 25 kHz
    [InlineData(2, 6198, 900)]       // and from 25 to 50 kHz
    public void SegmentEdgesLandOnTheMeasuredFrequencies(int band, int raw, int expectedKhz) =>
        Assert.Equal(expectedKhz, UH1H_AdfScale.ToKhz(band, raw));

    [Theory]
    [InlineData(12222, 443)]         // a 443 kHz beacon is heard here
    [InlineData(2450, 408)]          // and a 408 kHz one here
    public void MidBandReadsTheBeaconsCheckedInTheSimulator(int raw, int expectedKhz) =>
        Assert.Equal(expectedKhz, UH1H_AdfScale.ToKhz(1, raw));

    [Fact]
    public void MidBandReadsEveryWholeKilohertzAroundTheRefit()
    {
        // The refit must not squeeze a frequency out of the scale (no kink): every kHz from
        // 420 to 475 has raw positions that read it.
        var seen = new HashSet<int>();
        for (var raw = 6200; raw <= 16234; raw++) seen.Add(UH1H_AdfScale.ToKhz(1, raw));
        for (var khz = 420; khz <= 475; khz++) Assert.Contains(khz, seen);
    }

    [Fact]
    public void ReadingsRiseWithTheDialInsideASegment()
    {
        // 5510..8658 reads 200..210: halfway through is 205, not what a straight line across
        // the whole band would give.
        Assert.Equal(205, UH1H_AdfScale.ToKhz(0, (5510 + 8658) / 2));
    }

    [Fact]
    public void ScaleIsMonotonicOnEveryBand()
    {
        for (var band = 0; band <= 2; band++)
        {
            var previous = int.MinValue;
            for (var raw = 0; raw <= 65535; raw += 17)
            {
                var khz = UH1H_AdfScale.ToKhz(band, raw);
                Assert.True(khz >= previous, $"band {band}: raw {raw} read {khz} after {previous}");
                previous = khz;
            }
        }
    }

    [Fact]
    public void StraightInterpolationWouldBeWrongByTensOfKilohertz()
    {
        // Halfway along the low band's dial is not the 295 kHz a straight line gives.
        var linear = 190 + (400 - 190) * 32768 / 65535.0;
        Assert.True(Math.Abs(UH1H_AdfScale.ToKhz(0, 32768) - linear) >= 1);
    }

    [Theory]
    [InlineData(-5, 190)]
    [InlineData(70000, 400)]
    public void OutOfRangeRawIsClamped(int raw, int expectedKhz) =>
        Assert.Equal(expectedKhz, UH1H_AdfScale.ToKhz(0, raw));

    [Fact]
    public void StateReadsTheFrequencyThroughTheScale()
    {
        var state = new UH1H_State { adfBand = ADFBand.Mid, adfFreqPos = 12222 };

        Assert.Equal(443, state.AdfFrequency);
    }
}
