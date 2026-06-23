using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class A10C_Listener
{
    private void HandleLoadoutKey(Key key)
    {
        if (key == _nextPageKey)
        {
            Scratchpad.Clear();
            _currentPage = LANDING_PAGE;
            ComputeLanding();
            RenderLandingPage();
        }
        else if (key == _prevPageKey)
        {
            Scratchpad.Clear();
            _currentPage = TAKEOFF_PAGE;
            Compute();
            RenderTakeoffPage();
        }
        else if (key == _perfPageKey)
        {
            Scratchpad.Clear();
            _currentPage = DEFAULT_PAGE;
        }
    }

    // Called from OnDisplayTick() (timer thread) every 100ms while on this page.
    private void RenderLoadoutPage()
    {
        var c = GetCompositor(LOADOUT_PAGE);
        c.Clear();

        c.Line(0).Small().White().Write("2/3").Centered("LOADOUT");

        var data = _lastExportData;
        if (data == null)
        {
            // Write() establishes column 0 before Centered so it knows the line width.
            c.Line(4).Small().White().Write("").Centered("NO LIVE DATA");
            c.Line(5).Small().White().Write("").Centered("ENABLE LIVE EXPORT");
            c.Line(6).Small().White().Write("").Centered("IN OPTIONS");
            return;
        }

        // Render stations sorted by DCS id — whatever numbering DCS uses.
        var stations = data.Stations ?? new List<StationData>();
        int line = 1;
        foreach (var st in stations.OrderBy(s => s.Id))
        {
            if (line > 11) break;
            if (st.Count > 0 && st.Name != null)
                WriteLoadoutLine(c, line, st.Id, st.Name, st.Count);
            else
                c.Line(line).Small().White().Write($"{st.Id,2} ------");
            line++;
        }

        // Line 12: cannon (GAU-8), always shown.
        if (data.Cannon is int shells && shells > 0)
        {
            int cannonWt = A10CWeightTable.CannonWeight(shells);
            string cannonName = $"GAU-8 {shells}rds";
            string nameCol = cannonName.Length > 16 ? cannonName[..16] : cannonName.PadRight(16);
            c.Line(12).Small().White().Write($"CN {nameCol} {cannonWt,4}");
        }
        else
            c.Line(12).Small().White().Write("CN ------");

        c.Line(13).Cyan().Write(Scratchpad.DisplayText);
    }

    // "{id,2} {name,16} {weight,4}" — 24 chars
    private static void WriteLoadoutLine(Compositor c, int line, int pylon, string name, int count)
    {
        string id = pylon > 0 ? $"{pylon,2}" : "CN";

        string countPrefix = count > 1 ? $"{count}x" : "";
        string merged = $"{countPrefix}{name}";

        int? wt = A10CWeightTable.StationWeight(name, count);
        string wtCol = wt.HasValue ? $"{wt.Value,4}" : "   ?";

        string nameCol = merged.Length > 16 ? merged[..16] : merged.PadRight(16);
        c.Line(line).Small().White().Write($"{id} {nameCol} {wtCol}");
    }
}
