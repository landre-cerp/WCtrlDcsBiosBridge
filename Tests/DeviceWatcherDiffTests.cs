using WCtrlDcsBiosBridge.Services;
using WwDevicesDotNet;
using Xunit;

namespace WCtrlDcsBiosBridge.Tests;

/// <summary>
/// Exercises the hot-plug reconciliation diff (<see cref="DeviceWatcher.Diff"/>) with
/// in-memory device lists. This is the correctness core of the USB hot-plug feature, and
/// it runs with no hardware attached — only <see cref="DeviceIdentifier"/> value-equality.
/// </summary>
public class DeviceWatcherDiffTests
{
    // Two MCDUs differ by DeviceUser (Captain vs First Officer) and so are distinguishable —
    // this is the common dual-CDU CH-47 setup.
    private static DeviceIdentifier Mcdu(DeviceUser user) =>
        new($"MCDU {user}", 0x4098, 0xBB36, Device.WinctrlMcdu, user, DeviceType.AirbusA320Mcdu);

    // A panel that reports no per-seat identity (e.g. AGP32 / FCU / PAP3): two of the same
    // model are value-identical.
    private static DeviceIdentifier Anonymous() =>
        new("AGP32", 0x4098, 0xBC27, Device.WinctrlAgp32, DeviceUser.NotApplicable, DeviceType.Agp32);

    [Fact]
    public void NoChange_ProducesNoEvents()
    {
        var a = Mcdu(DeviceUser.Captain);
        var (arrived, removed) = DeviceWatcher.Diff(known: new[] { a }, current: new[] { a });

        Assert.Empty(arrived);
        Assert.Empty(removed);
    }

    [Fact]
    public void NewDevice_IsReportedAsArrived()
    {
        var capt = Mcdu(DeviceUser.Captain);
        var fo = Mcdu(DeviceUser.FirstOfficer);

        var (arrived, removed) = DeviceWatcher.Diff(known: new[] { capt }, current: new[] { capt, fo });

        Assert.Equal(new[] { fo }, arrived);
        Assert.Empty(removed);
    }

    [Fact]
    public void UnpluggedDistinctDevice_IsReportedAsRemoved()
    {
        var capt = Mcdu(DeviceUser.Captain);
        var fo = Mcdu(DeviceUser.FirstOfficer);

        var (arrived, removed) = DeviceWatcher.Diff(known: new[] { capt, fo }, current: new[] { fo });

        Assert.Empty(arrived);
        Assert.Equal(new[] { capt }, removed);
    }

    [Fact]
    public void SimultaneousArriveAndRemove_AreBothReported()
    {
        var capt = Mcdu(DeviceUser.Captain);
        var fo = Mcdu(DeviceUser.FirstOfficer);

        var (arrived, removed) = DeviceWatcher.Diff(known: new[] { capt }, current: new[] { fo });

        Assert.Equal(new[] { fo }, arrived);
        Assert.Equal(new[] { capt }, removed);
    }

    /// <summary>
    /// Documents (and locks in) the known limitation: two value-identical panels cannot be
    /// told apart, so unplugging one of a matched pair produces no removal event. If this
    /// ever becomes distinguishable (e.g. by serial/path), this test should be updated.
    /// </summary>
    [Fact]
    public void IdenticalPair_UnplugOne_ProducesNoRemoval_KnownLimitation()
    {
        var first = Anonymous();
        var second = Anonymous();   // distinct object, value-equal to first

        var (arrived, removed) = DeviceWatcher.Diff(known: new[] { first, second }, current: new[] { first });

        Assert.Empty(arrived);
        Assert.Empty(removed);      // the unplugged twin is invisible to the diff
    }
}
