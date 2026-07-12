using System.Runtime.InteropServices;
using Microsoft.UI.Windowing;
using Windows.Graphics;

namespace WCtrlDcsBiosBridge.Services;

/// <summary>
/// AppWindow.Resize()/PreferredMinimumWidth take physical pixels, not the DIP-like
/// logical units WPF sizes were written in — on a scaled display (125%/150%/200%),
/// requesting a raw "640x620" renders far smaller than intended and clips content.
/// This scales logical sizes by the window's actual DPI before applying them.
/// </summary>
public static class WindowSizing
{
    [DllImport("user32.dll")]
    private static extern int GetDpiForWindow(IntPtr hwnd);

    public static double GetScale(IntPtr hwnd) => GetDpiForWindow(hwnd) / 96.0;

    public static void Resize(AppWindow appWindow, IntPtr hwnd, int logicalWidth, int logicalHeight)
    {
        var scale = GetScale(hwnd);
        appWindow.Resize(new SizeInt32((int)(logicalWidth * scale), (int)(logicalHeight * scale)));
    }

    public static void CenterOnDisplay(AppWindow appWindow)
    {
        var displayArea = DisplayArea.GetFromWindowId(appWindow.Id, DisplayAreaFallback.Primary);
        var x = (displayArea.WorkArea.Width - appWindow.Size.Width) / 2;
        var y = (displayArea.WorkArea.Height - appWindow.Size.Height) / 2;
        appWindow.Move(new PointInt32(x, y));
    }
}
