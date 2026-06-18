using NLog;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WCtrlDcsBiosBridge.Services;

namespace WCtrlDcsBiosBridge.Aircrafts.UH1H
{
    public enum ADFMode
    {
        OFF = 0,
        ADF = 1,
        ANT = 2,
        LOOP = 3
    }

    public enum UHFFunctionDial
    {
        OFF =0, 
        TR = 1,
        TR_G = 2,
        ADF = 3,

    }

    public enum UHFModeDial
    {
        
        PRESET = 0,
        MAN = 1,
        GDXMIT = 2,
        
    }

    internal class UH1H_state
    {
        public int adfBand = 0;
        public int adfFreqPos = 0;
        public ADFMode adfMode = ADFMode.OFF;

        public UHFFunctionDial UHF_ARC251_Mode = UHFFunctionDial.OFF;
        public string UHF_ARC251 = "UHF_ARC251";
        public UHFModeDial UHFModeDial = UHFModeDial.MAN;

        public int UHF_ARC251_Preset = 1;

        private Dictionary<int, (int, int)> bands = new Dictionary<int, (int, int)>
        {
            { 0, (190, 400) },
            { 1, (400, 850) },
            { 2, (850, 1800) }
        };
        private double adfFreq;

        public double Frequency
        {
            get => getFrequency();
        }
        double Map(double value, double inMin, double inMax, double outMin, double outMax)
        {
            return (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
        }
        double getFrequency()
        {
            bands.TryGetValue(adfBand, out var band);
            return Map(adfFreqPos, 0, 65535, band.Item1, band.Item2);

        }
    }
}
