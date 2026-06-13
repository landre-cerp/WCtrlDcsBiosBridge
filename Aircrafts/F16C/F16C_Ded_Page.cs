using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;
using System.Text;
using WwDevicesDotNet;

namespace WWCduDcsBiosBridge.Aircrafts;

internal class F16C_Ded_Page
{
    private readonly DCSBIOSOutput?[] _dedMainLines = new DCSBIOSOutput?[5];
    private readonly DCSBIOSOutput?[] _dedSecLines = new DCSBIOSOutput?[5];
    private readonly DCSBIOSOutput?[] _dedFormats = new DCSBIOSOutput?[5];

    private readonly string[] _dedMainText = new string[5];
    private readonly string[] _dedFormatText = new string[5];
    private readonly string[] _dedRenderedMain = new string[5];
    private readonly string[] _dedRenderedFmt = new string[5];

    private static readonly Dictionary<char, char> DedCharMap = new()
    {
        { '\x00', ' '  },
        { '\x01', '*'  },
        { '\x02', '>'  },
        { '\x03', '<'  },
        { '\x04', '^'  },
        { '\x05', 'v'  },
        { '\x06', '-'  },
        { '\x08', ' '  },
        { '\x0B', '['  },
        { '\x0C', ']'  },
        { '\x0E', '+'  },
        { '\x0F', '='  },
        { '\x10', '*'  },
        { '\x11', 'T'  },
        { '\x12', 'I'  },
        { '\x14', '%'  },
        { '\x16', '_'  },
        { '©', '^'  },
        { '®', 'D'  },
        { '¡', '↑'  },
        { 'i', '↓'  },
        { '±', '_'  },
        { '«', '<'  },
        { '»', '>'  },
        { '°', '°'  },
        { 'a', 'Δ'  },
    };

    public F16C_Ded_Page()
    {
        for (int i = 0; i < 5; i++)
        {
            _dedMainText[i] = string.Empty;
            _dedFormatText[i] = string.Empty;
            _dedRenderedMain[i] = "\xFF";
            _dedRenderedFmt[i] = "\xFF";
        }
    }

    public void InitializeControls()
    {
        _dedMainLines[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_1");
        _dedMainLines[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_2");
        _dedMainLines[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_3");
        _dedMainLines[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_4");
        _dedMainLines[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_LINE_5");

        _dedSecLines[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L1");
        _dedSecLines[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L2");
        _dedSecLines[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L3");
        _dedSecLines[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L4");
        _dedSecLines[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L5");

        _dedFormats[0] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L1_FORMAT");
        _dedFormats[1] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L2_FORMAT");
        _dedFormats[2] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L3_FORMAT");
        _dedFormats[3] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L4_FORMAT");
        _dedFormats[4] = DCSBIOSControlLocator.GetStringDCSBIOSOutput("DED_L5_FORMAT");
    }

    public void RegisterControls(
        Action<DCSBIOSOutput?, Action<uint>> register,
        Action<DCSBIOSOutput?, Action<string>> registerString,
        Func<Compositor> compositor)
    {
        for (int i = 0; i < 5; i++)
        {
            int idx = i;
            registerString(_dedMainLines[idx], s => { _dedMainText[idx] = s; Render(compositor()); });
            registerString(_dedFormats[idx], s => { _dedFormatText[idx] = s; Render(compositor()); });
        }
    }

    public void Render(Compositor output)
    {
        for (int i = 0; i < 5; i++)
        {
            string text = NormaliseDedLine(ApplyDedCharacterMap(_dedMainText[i]));
            string fmt = _dedFormatText[i];

            if (text == _dedRenderedMain[i] && fmt == _dedRenderedFmt[i])
            {
                continue;
            }

            _dedRenderedMain[i] = text;
            _dedRenderedFmt[i] = fmt;

            RenderDedLine(output, i, text, fmt);
        }

        for (int r = 5; r < Metrics.Lines - 1; r++)
        {
            output.Line(r).BGBlack().Green().Write(new string(' ', Metrics.Columns));
        }

        output.Line(Metrics.Lines - 1).BGBlack().Green()
            .Write(new string(' ', Metrics.Columns - 5))
            .Amber().Write("(DED)");
    }

    public void InvalidateCache()
    {
        for (int i = 0; i < 5; i++)
        {
            _dedRenderedMain[i] = "\xFF";
            _dedRenderedFmt[i] = "\xFF";
        }
    }

    private static void RenderDedLine(Compositor output, int row, string text, string fmt)
    {
        string safeFmt = fmt.PadRight(text.Length);

        output.Line(row);

        int pos = 0;
        while (pos < text.Length)
        {
            bool isInverse = safeFmt[pos] == 'i';
            int start = pos;

            while (pos < text.Length && (safeFmt[pos] == 'i') == isInverse)
            {
                pos++;
            }

            string segment = text[start..pos];

            if (isInverse)
            {
                output.Black().BGGreen().Write(segment);
            }
            else
            {
                output.Green().BGBlack().Write(segment);
            }
        }
    }

    private static string ApplyDedCharacterMap(string raw)
    {
        if (string.IsNullOrEmpty(raw))
        {
            return string.Empty;
        }

        var sb = new StringBuilder(raw.Length);
        foreach (char c in raw)
        {
            sb.Append(DedCharMap.TryGetValue(c, out char mapped) ? mapped : c);
        }

        return sb.ToString();
    }

    private static string NormaliseDedLine(string text)
    {
        return text.Length >= Metrics.Columns
            ? text[..Metrics.Columns]
            : text.PadRight(Metrics.Columns);
    }
}
