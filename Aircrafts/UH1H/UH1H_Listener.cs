using DCS_BIOS.ControlLocator;
using DCS_BIOS.Serialized;

namespace WCtrlDcsBiosBridge.Aircrafts.UH1H
{

    internal class UH1H_Listener : AircraftListener
    {
        private UH1H.UH1H_state _state = new();

        // ADF 
        private DCSBIOSOutput? _ADF_BAND; // 0-2
        private DCSBIOSOutput? _ADF_FREQ; // 0-65535
        private DCSBIOSOutput? _ADF_BFO_SW; // 0-1
        private DCSBIOSOutput? _ADF_MODE; // 0-3 => OFF ADF ANT LOOP 

        // UHF ARC-251
        private DCSBIOSOutput? _UHF_FREQ;
        private DCSBIOSOutput? _UHF_FUNCTION;
        private DCSBIOSOutput? _UHF_MODE; // Man Preset Gd xmit
        private DCSBIOSOutput? _UHF_PRESET;

        public UH1H_Listener(UserOptions options) :base(AircraftRegistry.UH1H, options)
        {
            
        }

        protected override void InitializeDcsBiosOutputs()
        {
            _ADF_BAND = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ADF_BAND");
            _ADF_FREQ = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ADF_FREQ");
            _ADF_BFO_SW = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ADF_BFO_SW");
            _ADF_MODE = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("ADF_MODE");

            _UHF_FREQ = DCSBIOSControlLocator.GetStringDCSBIOSOutput("UHF_FREQ");
            _UHF_FUNCTION = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("UHF_FUNCTION");
            _UHF_MODE = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("UHF_MODE");
            _UHF_PRESET = DCSBIOSControlLocator.GetUIntDCSBIOSOutput("UHF_PRESET");
        }

        protected override void RegisterCduControls()
        {
            Register( _ADF_MODE, (value) =>
            {
                _state.adfMode = (ADFMode)value;
                UpdateAdfDisplay();
            });
            Register(_ADF_BAND, (value) =>
            {
                _state.adfBand = (int)value;
                UpdateAdfDisplay();
            });
            Register(_ADF_FREQ, (value) =>
            {
                _state.adfFreqPos = (int)value;
                UpdateAdfDisplay();
            });
            RegisterString(_UHF_FREQ, (value) =>
            {
                _state.UHF_ARC251 = value;
                UpdateUHFDisplay();
            });
            Register(_UHF_FUNCTION, (value) =>
            {
                _state.UHF_ARC251_Mode = (UHFFunctionDial)value;
                UpdateUHFDisplay();
            });
            Register(_UHF_MODE, (value) =>
            {
                _state.UHFModeDial = (UHFModeDial)value;
            });
            Register(_UHF_PRESET, (value) =>
            {
                _state.UHF_ARC251_Preset = (int)value+1;
                UpdateUHFDisplay();
            });
        }

        protected override void RegisterFrontpanelControls()
        {
        }

        protected void UpdateAdfDisplay()
        {
            if (_state.adfMode == ADFMode.OFF)
            {
                GetCompositor(_currentPage).Line(1).ClearRow().WriteLine($"ADF: OFF");
                return;
            }
            GetCompositor(_currentPage).Line(1).ClearRow().WriteLine($"ADF: {_state.Frequency:F2}");
        }

        protected void UpdateUHFDisplay()
        {
            if (_state.UHF_ARC251_Mode == UHFFunctionDial.OFF)
            {
                GetCompositor(_currentPage).Line(2).ClearRow().WriteLine($"UHF: OFF");
                return;
            }
            if (_state.UHFModeDial == UHFModeDial.PRESET)
            {
                GetCompositor(_currentPage).Line(2).ClearRow().WriteLine($"UHF: {_state.UHF_ARC251_Mode} {_state.UHFModeDial} {_state.UHF_ARC251_Preset}");
                return;
            }
            GetCompositor(_currentPage).Line(2).ClearRow().WriteLine($"UHF: {_state.UHF_ARC251} {_state.UHF_ARC251_Mode} {_state.UHFModeDial}");
        }
    }

}