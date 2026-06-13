using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class FA18C_UFC_Page
{
    private DCSBIOSOutput? _optionDisplay1;
    private DCSBIOSOutput? _optionCueing1;
    private DCSBIOSOutput? _optionDisplay2;
    private DCSBIOSOutput? _optionCueing2;
    private DCSBIOSOutput? _optionDisplay3;
    private DCSBIOSOutput? _optionCueing3;
    private DCSBIOSOutput? _optionDisplay4;
    private DCSBIOSOutput? _optionCueing4;
    private DCSBIOSOutput? _optionDisplay5;
    private DCSBIOSOutput? _optionCueing5;
    private DCSBIOSOutput? _scratchpadNumber;
    private DCSBIOSOutput? _scratchpadString1;
    private DCSBIOSOutput? _scratchpadString2;

    string _cue1 = " ";
    string _cue2 = " ";
    string _cue3 = " ";
    string _cue4 = " ";
    string _cue5 = " ";
    string _option1 = "    ";
    string _option2 = "    ";
    string _option3 = "    ";
    string _option4 = "    ";
    string _option5 = "    ";
    string _scratchPadNumber = "        "; //8
    string _scratchPad1 = "  ";
    string _scratchPad2 = "  ";

    public void InitializeControls()
    {
        _optionDisplay1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_DISPLAY_1");
        _optionCueing1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_CUEING_1");
        _optionDisplay2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_DISPLAY_2");
        _optionCueing2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_CUEING_2");
        _optionDisplay3 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_DISPLAY_3");
        _optionCueing3 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_CUEING_3");
        _optionDisplay4 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_DISPLAY_4");
        _optionCueing4 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_CUEING_4");
        _optionDisplay5 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_DISPLAY_5");
        _optionCueing5 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_OPTION_CUEING_5");
        _scratchpadNumber = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_SCRATCHPAD_NUMBER_DISPLAY");
        _scratchpadString1 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_SCRATCHPAD_STRING_1_DISPLAY");
        _scratchpadString2 = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UFC_SCRATCHPAD_STRING_2_DISPLAY");
    }

    public void RegisterControls(Action<DCSBIOSOutput?, Action<string>> registerString, Action onUpdated)
    {
        registerString(_scratchpadNumber, s =>
        {
            if (string.Compare(s, "   pww0w") == 0) s = "   ERROR";
            if (string.Compare(s, _scratchPadNumber) != 0) _scratchPadNumber = s;
            onUpdated();
        });
        registerString(_scratchpadString1, s =>
        {
            if (string.Compare(s, _scratchPad1) != 0) _scratchPad1 = s;
            onUpdated();
        });
        registerString(_scratchpadString2, s =>
        {
            if (string.Compare(s, _scratchPad2) != 0) _scratchPad2 = s;
            onUpdated();
        });

        registerString(_optionDisplay1, s =>
        {
            if (string.Compare(s, _option1) != 0) _option1 = s;
            onUpdated();
        });
        registerString(_optionCueing1, s =>
        {
            if (string.Compare(s, _cue1) != 0) _cue1 = s;
            onUpdated();
        });
        registerString(_optionDisplay2, s =>
        {
            if (string.Compare(s, _option2) != 0) _option2 = s;
            onUpdated();
        });
        registerString(_optionCueing2, s =>
        {
            if (string.Compare(s, _cue2) != 0) _cue2 = s;
            onUpdated();
        });
        registerString(_optionDisplay3, s =>
        {
            if (string.Compare(s, _option3) != 0) _option3 = s;
            onUpdated();
        });
        registerString(_optionCueing3, s =>
        {
            if (string.Compare(s, _cue3) != 0) _cue3 = s;
            onUpdated();
        });
        registerString(_optionDisplay4, s =>
        {
            if (string.Compare(s, _option4) != 0) _option4 = s;
            onUpdated();
        });
        registerString(_optionCueing4, s =>
        {
            if (string.Compare(s, _cue4) != 0) _cue4 = s;
            onUpdated();
        });
        registerString(_optionDisplay5, s =>
        {
            if (string.Compare(s, _option5) != 0) _option5 = s;
            onUpdated();
        });
        registerString(_optionCueing5, s =>
        {
            if (string.Compare(s, _cue5) != 0) _cue5 = s;
            onUpdated();
        });
    }

    public void Render(Compositor output)
    {
        //  CDU layout: 24 chars wide, 12 rows (0-11)
        //
        //  Row 0:  "                      1/2"  (page number)
        //  Row 1:  "ss ss  nnnnnnn           "  (scratchpad: string1, string2, number)
        //  Row 2:  "                   c oooo"  (cue1 + option1)
        //  Row 3:  "                         "
        //  Row 4:  "                   c oooo"  (cue2 + option2)
        //  Row 5:  "                         "
        //  Row 6:  "                   c oooo"  (cue3 + option3)
        //  Row 7:  "                         "
        //  Row 8:  "                   c oooo"  (cue4 + option4)
        //  Row 9:  "                         "
        //  Row 10: "                   c oooo"  (cue5 + option5)
        //  Row 11: "                         "

        const string filler = "                   ";

        
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
