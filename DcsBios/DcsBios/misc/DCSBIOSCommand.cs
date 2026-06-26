using System;
using System.Collections.Generic;
using System.Linq;
using DCS_BIOS.Json;
using DCS_BIOS.Serialized;

namespace DCS_BIOS.misc
{
    /// <summary>
    /// Class for getting easy access to commands to send back relating to the DCS-BIOS control. E.g. "FLAPS_SWITCH INC\n"
    /// </summary>
    public class DCSBIOSCommand
    {
        private readonly List<DCSBIOSInputType> _supportedInterfaces = new();
        private readonly int _defaultVariableChangeValue = 0;
        public const string DCSBIOS_INCREMENT = "INC\n";
        public const string DCSBIOS_DECREMENT = "DEC\n";
        public const string DCSBIOS_TOGGLE = "TOGGLE\n";
        private readonly string _controlId;

        public DCSBIOSCommand(DCSBIOSControl dcsbiosControl)
        {
            _controlId = dcsbiosControl.Identifier;

            foreach (var dcsbiosControlInput in dcsbiosControl.Inputs.Where(dcsbiosControlInput => 
                         dcsbiosControlInput.GetInputInterface().Interface == DCSBIOSInputType.VARIABLE_STEP && dcsbiosControlInput.SuggestedStep is >= 0))
            {
                if (dcsbiosControlInput.SuggestedStep == null) continue;

                _defaultVariableChangeValue = (int)dcsbiosControlInput.SuggestedStep;
            }

            foreach (var dcsbiosControlInput in dcsbiosControl.Inputs)
            {
                _supportedInterfaces.Add(dcsbiosControlInput.GetInputInterface().Interface);
            }
        }

        public string ControlId
        {
            get => _controlId;
        }

        public string ControlIdWithSpace
        {
            get => _controlId + " ";
        }

        public string GetIncCommand()
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.FIXED_STEP) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.FIXED_STEP}");

            return $"{_controlId} {DCSBIOS_INCREMENT}";
        }

        public string GetDecCommand()
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.FIXED_STEP) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.FIXED_STEP}");

            return $"{_controlId} {DCSBIOS_DECREMENT}";
        }

        public string GetActionCommand()
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.ACTION) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.ACTION}");

            return $"{_controlId} {DCSBIOS_TOGGLE}";
        }

        public string GetSetStateCommand(uint value)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.SET_STATE) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.SET_STATE}");

            return $"{_controlId} {value}\n";
        }

        public string GetSetStateCommand(bool increment)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.SET_STATE) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.SET_STATE}");

            return $"{_controlId} {(increment ? DCSBIOS_INCREMENT : DCSBIOS_DECREMENT)}";
        }

        public string GetVariableCommand(int value)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.VARIABLE_STEP) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.VARIABLE_STEP}");

            return $"{_controlId} {(value > 0 ? "+" + value : value.ToString())}\n";
        }

        public string GetVariableCommand(int value, bool decrement)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.VARIABLE_STEP) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.VARIABLE_STEP}");

            if (decrement && value > 0)
            {
                value *= -1;
            }

            var s = value > 0 ? "+" + value : value.ToString();
            return $"{_controlId} {s}\n";
        }

        public string GetVariableDefaultCommand(bool increment)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.VARIABLE_STEP) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.VARIABLE_STEP}");

            var changeValue = increment ? "+" + _defaultVariableChangeValue : "-" + _defaultVariableChangeValue;
            return $"{_controlId} {changeValue}\n";
        }

        public string GetStringCommand(string parameters)
        {
            if (_supportedInterfaces.Any(o => o == DCSBIOSInputType.SET_STRING) == false)
                throw new ArgumentException($"DCSBIOSCommand: {_controlId} does not have {DCSBIOSInputType.SET_STRING}");
            
            return $"{_controlId} {parameters}\n";
        }
    }
}
