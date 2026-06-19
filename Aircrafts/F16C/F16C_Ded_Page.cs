using System.Text;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class F16C_Listener
{
    private readonly string[] _dedMainText   = new string[5];
    private readonly string[] _dedFormatText = new string[5];
    private readonly string[] _dedRenderedMain = new[] { "\xFF", "\xFF", "\xFF", "\xFF", "\xFF" };
    private readonly string[] _dedRenderedFmt  = new[] { "\xFF", "\xFF", "\xFF", "\xFF", "\xFF" };

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

    private void RegisterDedControls()
    {
        for (int i = 0; i < 5; i++)
        {
            int idx = i;
            RegisterStr($"DED_LINE_{i + 1}",    s => { _dedMainText[idx]   = s; RenderDed(); });
            RegisterStr($"DED_L{i + 1}_FORMAT", s => { _dedFormatText[idx] = s; RenderDed(); });
        }
    }

    private void InvalidateDedCache()
    {
        for (int i = 0; i < 5; i++)
        {
            _dedRenderedMain[i] = "\xFF";
            _dedRenderedFmt[i]  = "\xFF";
        }
    }

    private void RenderDed()
    {
        var output = GetCompositor(DEFAULT_PAGE);

        for (int i = 0; i < 5; i++)
        {
            string text = NormaliseDedLine(ApplyDedCharacterMap(_dedMainText[i]));
            string fmt  = _dedFormatText[i];

            if (text == _dedRenderedMain[i] && fmt == _dedRenderedFmt[i])
                continue;

            _dedRenderedMain[i] = text;
            _dedRenderedFmt[i]  = fmt;

            RenderDedLine(output, i, text, fmt);
        }

        for (int r = 5; r < Metrics.Lines - 1; r++)
            output.Line(r).BGBlack().Green().Write(new string(' ', Metrics.Columns));

        output.Line(Metrics.Lines - 1).BGBlack().Green()
            .Write(new string(' ', Metrics.Columns - 5))
            .Amber().Write("(DED)");
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
                pos++;

            string segment = text[start..pos];
            if (isInverse)
                output.Black().BGGreen().Write(segment);
            else
                output.Green().BGBlack().Write(segment);
        }
    }

    private static string ApplyDedCharacterMap(string raw)
    {
        if (string.IsNullOrEmpty(raw))
            return string.Empty;

        var sb = new StringBuilder(raw.Length);
        foreach (char c in raw)
            sb.Append(DedCharMap.TryGetValue(c, out char mapped) ? mapped : c);

        return sb.ToString();
    }

    private static string NormaliseDedLine(string text) =>
        text.Length >= Metrics.Columns ? text[..Metrics.Columns] : text.PadRight(Metrics.Columns);
}
