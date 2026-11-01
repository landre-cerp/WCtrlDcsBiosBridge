using DCS_BIOS.EventArgs;
using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

internal partial class FA18C_Listener : AircraftListener
{
    private const string IFEI_PAGE = "IFEI";

    uint _lightMode = 0; // 2=NVG, 1=NITE, 0=DAY

    private readonly Key _nextPageKey;
    private readonly Key _prevPageKey;
    private readonly bool _useGreen;

    // RenderUfc/RenderIfei run on the DCS-BIOS receive thread and on the MCDU key thread, and a
    // page's Screen keeps a shared cursor/colour. Unserialised, a fast page switch interleaves
    // them and writes one page's title onto another's row.
    private readonly object _renderLock = new();

    public FA18C_Listener(UserOptions options)
        : base(AircraftRegistry.FA18C, options)
    {
        _useGreen = options.FA18C.UseGreen;
        _nextPageKey = Enum.TryParse<Key>(options.FA18C.ShowIfeiKey, out var nextKey)
            ? nextKey
            : Key.NextPage;
        _prevPageKey = Enum.TryParse<Key>(options.FA18C.ShowUfcKey, out var prevKey)
            ? prevKey
            : Key.PrevPage;

        AddNewPage(IFEI_PAGE);
        
    }

    private void HandleKeyDown(object? sender, KeyEventArgs e)
    {
               
        if (e.Key == _nextPageKey) { _currentPage = IFEI_PAGE;   RenderIfei(); }
        else if (e.Key == _prevPageKey) { _currentPage = DEFAULT_PAGE; RenderUfc(); }
        
    }

    protected override void RegisterCduControls()
    {
        if (CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
            CduDevice.KeyDown += HandleKeyDown;
        }

        RegisterUInt("COCKKPIT_LIGHT_MODE_SW", v => _lightMode = v);
        RegisterUfcControls();
        RegisterIfeiControls();
    }

    protected override void RegisterFrontpanelControls()
    {
        // Gear and flap lights are declared in LedDefaults.

        RegisterIfeiFrontPanelControls();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing && CduDevice != null)
        {
            CduDevice.KeyDown -= HandleKeyDown;
        }
        base.Dispose(disposing);
    }
}
