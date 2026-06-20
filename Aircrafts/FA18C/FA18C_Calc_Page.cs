using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class FA18C_Listener
{
    private string? _calcA;
    private string? _calcB;
    private string? _calcResult;

    private void RegisterCalcControls()
    {
        Scratchpad.Changed += (_, _) =>
        {
            if (_currentPage == CALC_PAGE)
                RenderCalcPage();
        };
    }

    private void HandleCalcKey(Key key)
    {
        if (key == Key.LineSelectLeft1)
        {
            Scratchpad.CommitToField(ref _calcA);
            ComputeCalcResult();
            RenderCalcPage();
        }
        else if (key == Key.LineSelectLeft2)
        {
            Scratchpad.CommitToField(ref _calcB);
            ComputeCalcResult();
            RenderCalcPage();
        }
        else if (key == _nextPageKey)
        {
            Scratchpad.Clear();
            _currentPage = IFEI_PAGE;
            RenderIfei();
        }
        else if (key == _prevPageKey || key == Key.Perf)
        {
            Scratchpad.Clear();
            _currentPage = DEFAULT_PAGE;
            RenderUfc();
        }
    }

    private void ComputeCalcResult()
    {
        if (_calcA != null && _calcB != null
            && double.TryParse(_calcA, System.Globalization.NumberStyles.Any,
                               System.Globalization.CultureInfo.InvariantCulture, out var a)
            && double.TryParse(_calcB, System.Globalization.NumberStyles.Any,
                               System.Globalization.CultureInfo.InvariantCulture, out var b))
        {
            _calcResult = (a + b).ToString("G8", System.Globalization.CultureInfo.InvariantCulture);
        }
        else
        {
            _calcResult = null;
        }
    }

    private void RenderCalcPage()
    {
        var c = GetCompositor(CALC_PAGE);
        c.Line(0).White().Centered("CALC").Column(21).Write("1/1");

        c.Line(1).Small().White().Write("OPERAND A");
        c.Line(2).Large().Cyan().Write(BoxField(_calcA, 8));

        c.Line(3).Small().White().Write("OPERAND B");
        c.Line(4).Large().Cyan().Write(BoxField(_calcB, 8));

        c.Line(5).ClearRow();
        c.Line(6).Small().White().Write("SUM");
        c.Line(7).ClearRow().Large().Green().Write(_calcResult ?? "--------");

        for (int i = 8; i <= 12; i++) c.Line(i).ClearRow();

        c.Line(13).ClearRow().Cyan().Write(Scratchpad.DisplayText);
    }
}
