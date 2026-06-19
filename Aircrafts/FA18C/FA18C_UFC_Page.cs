using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class FA18C_Listener
{
    private string _cue1 = " ";  private string _cue2 = " ";  private string _cue3 = " ";
    private string _cue4 = " ";  private string _cue5 = " ";
    private string _option1 = "    "; private string _option2 = "    "; private string _option3 = "    ";
    private string _option4 = "    "; private string _option5 = "    ";
    private string _scratchPadNumber = "        ";
    private string _scratchPad1 = "  ";
    private string _scratchPad2 = "  ";

    private void RegisterUfcControls()
    {
        RegisterStr("UFC_SCRATCHPAD_NUMBER_DISPLAY", s =>
        {
            if (string.Compare(s, "   pww0w") == 0) s = "   ERROR";
            if (string.Compare(s, _scratchPadNumber) != 0) _scratchPadNumber = s;
            RenderUfc();
        });
        RegisterStr("UFC_SCRATCHPAD_STRING_1_DISPLAY", s =>
        {
            if (string.Compare(s, _scratchPad1) != 0) _scratchPad1 = s;
            RenderUfc();
        });
        RegisterStr("UFC_SCRATCHPAD_STRING_2_DISPLAY", s =>
        {
            if (string.Compare(s, _scratchPad2) != 0) _scratchPad2 = s;
            RenderUfc();
        });

        RegisterStr("UFC_OPTION_DISPLAY_1", s => { if (string.Compare(s, _option1) != 0) _option1 = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_CUEING_1",  s => { if (string.Compare(s, _cue1)    != 0) _cue1    = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_DISPLAY_2", s => { if (string.Compare(s, _option2) != 0) _option2 = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_CUEING_2",  s => { if (string.Compare(s, _cue2)    != 0) _cue2    = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_DISPLAY_3", s => { if (string.Compare(s, _option3) != 0) _option3 = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_CUEING_3",  s => { if (string.Compare(s, _cue3)    != 0) _cue3    = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_DISPLAY_4", s => { if (string.Compare(s, _option4) != 0) _option4 = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_CUEING_4",  s => { if (string.Compare(s, _cue4)    != 0) _cue4    = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_DISPLAY_5", s => { if (string.Compare(s, _option5) != 0) _option5 = s; RenderUfc(); });
        RegisterStr("UFC_OPTION_CUEING_5",  s => { if (string.Compare(s, _cue5)    != 0) _cue5    = s; RenderUfc(); });
    }

    private void RenderUfc()
    {
        const string filler = "                   ";
        var output = GetCompositor(DEFAULT_PAGE);

        output.Line(0).ClearRow().Yellow().Centered("F/A-18C UFC").Column(21).Write("1/2");
        output.Line(1).WriteLine(string.Format("{0,2}{1,2}{2,8}", _scratchPad1, _scratchPad2, _scratchPadNumber));
        output.Line(2).WriteLine(string.Format("{2,19}{0,1}{1,4}", _cue1, _option1, filler));
        output.Line(3).ClearRow();
        output.Line(4).WriteLine(string.Format("{2,19}{0,1}{1,4}", _cue2, _option2, filler));
        output.Line(5).ClearRow();
        output.Line(6).WriteLine(string.Format("{2,19}{0,1}{1,4}", _cue3, _option3, filler));
        output.Line(7).ClearRow();
        output.Line(8).WriteLine(string.Format("{2,19}{0,1}{1,4}", _cue4, _option4, filler));
        output.Line(9).ClearRow();
        output.Line(10).WriteLine(string.Format("{2,19}{0,1}{1,4}", _cue5, _option5, filler));
        output.Line(11).ClearRow();
    }
}
