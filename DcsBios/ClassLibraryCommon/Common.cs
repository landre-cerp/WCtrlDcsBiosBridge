namespace ClassLibraryCommon
{
    using NLog;
    using System;
    using System.Collections.Generic;
    using System.IO;
    using System.Linq;
    using System.Security.Cryptography;
    using System.Text;

    [Flags]
    public enum EmulationMode
    {
        DCSBIOSInputEnabled = 1,
        DCSBIOSOutputEnabled = 2,
        KeyboardEmulationOnly = 4,
        SRSEnabled = 8,
        NS430Enabled = 16
    }

    public static class Common
    {
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();
        private static int _emulationModesFlag;

        public static Tuple<bool, bool> CheckJSONDirectory(string jsonDirectory)
        {
            jsonDirectory = Environment.ExpandEnvironmentVariables(jsonDirectory);

            if (string.IsNullOrEmpty(jsonDirectory) || !Directory.Exists(jsonDirectory))
            {
                return new Tuple<bool, bool>(false, false);
            }

            var files = Directory.EnumerateFiles(jsonDirectory);
            var jsonFound = files.Count(filename => filename.ToLower().EndsWith(".json")) >= 10;

            return new Tuple<bool, bool>(true, jsonFound);
        }

        public static APIModeEnum APIModeUsed { get; set; } = 0;

        public static string GetHex(uint i, bool includePrefix = true, bool lowercase = true, bool padLengthTo4 = true)
        {
            return GetHex((int)i, includePrefix, lowercase, padLengthTo4);
        }

        public static string GetHex(int i, bool includePrefix = true, bool lowercase = true, bool padLengthTo4 = true)
        {
            var formatter = lowercase ? "x" : "X";
            var s = i.ToString(formatter);
            if (padLengthTo4)
            {
                s = s.PadLeft(4, '0');
            }
            if (includePrefix) s = s.Insert(0, "0x");
            return s;
        }

        public static string RemoveCurlyBrackets(string s)
        {
            return string.IsNullOrEmpty(s) ? null : s.Replace("{", "").Replace("}", "");
        }

        public static string RemoveLControl(string keySequence)
        {
            return true switch
            {
                _ when keySequence.Contains("RMENU + LCONTROL") => keySequence.Replace("+ LCONTROL", string.Empty),
                _ when keySequence.Contains("LCONTROL + RMENU") => keySequence.Replace("LCONTROL +", string.Empty),
                _ => keySequence
            };
        }

        public static readonly List<GamingPanelSkeleton> GamingPanelSkeletons = new()
        {
            new GamingPanelSkeleton(GamingPanelVendorEnum.Saitek, GamingPanelEnum.PZ55SwitchPanel),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Saitek, GamingPanelEnum.PZ69RadioPanel),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Saitek, GamingPanelEnum.PZ70MultiPanel),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Saitek, GamingPanelEnum.BackLitPanel),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Saitek, GamingPanelEnum.TPM),
            new GamingPanelSkeleton(GamingPanelVendorEnum.MadCatz, GamingPanelEnum.FarmingPanel),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckMini),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckMiniV2),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeck),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckV2),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckMK2),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckXL),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckXLRev2),
            new GamingPanelSkeleton(GamingPanelVendorEnum.CockpitMaster, GamingPanelEnum.CDU737),
            new GamingPanelSkeleton(GamingPanelVendorEnum.Elgato, GamingPanelEnum.StreamDeckPlus)
        };

        private static void ValidateEmulationModeFlag()
        {
            if (!IsEmulationModesFlagSet(EmulationMode.KeyboardEmulationOnly)) return;
            if (IsEmulationModesFlagSet(EmulationMode.DCSBIOSOutputEnabled) ||
                IsEmulationModesFlagSet(EmulationMode.DCSBIOSInputEnabled))
            {
                throw new Exception($"Invalid emulation modes flag : {_emulationModesFlag}");
            }
        }

        public static void SetEmulationModesFlag(int flag)
        {
            _emulationModesFlag = flag;
            ValidateEmulationModeFlag();
        }

        public static int GetEmulationModesFlag()
        {
            ValidateEmulationModeFlag();
            return _emulationModesFlag;
        }

        public static void SetEmulationModes(EmulationMode emulationMode)
        {
            _emulationModesFlag |= (int)emulationMode;
            ValidateEmulationModeFlag();
        }

        public static bool IsEmulationModesFlagSet(EmulationMode flagValue)
        {
            return (_emulationModesFlag & (int)flagValue) > 0;
        }

        public static void ClearEmulationModesFlag(EmulationMode flagValue)
        {
            _emulationModesFlag &= ~(int)flagValue;
        }

        public static void ResetEmulationModesFlag()
        {
            _emulationModesFlag = 0;
        }

        public static bool NoDCSBIOSEnabled()
        {
            ValidateEmulationModeFlag();
            return !IsEmulationModesFlagSet(EmulationMode.DCSBIOSInputEnabled) && !IsEmulationModesFlagSet(EmulationMode.DCSBIOSOutputEnabled);
        }

        public static bool KeyEmulationOnly()
        {
            ValidateEmulationModeFlag();
            return IsEmulationModesFlagSet(EmulationMode.KeyboardEmulationOnly);
        }

        public static bool FullDCSBIOSEnabled()
        {
            ValidateEmulationModeFlag();
            return IsEmulationModesFlagSet(EmulationMode.DCSBIOSOutputEnabled) && IsEmulationModesFlagSet(EmulationMode.DCSBIOSInputEnabled);
        }

        public static bool PartialDCSBIOSEnabled()
        {
            ValidateEmulationModeFlag();
            return IsEmulationModesFlagSet(EmulationMode.DCSBIOSOutputEnabled) || IsEmulationModesFlagSet(EmulationMode.DCSBIOSInputEnabled);
        }

        public static string GetMd5Hash(string input)
        {
            var md5 = MD5.Create();
            var data = md5.ComputeHash(Encoding.UTF8.GetBytes(input));
            var sBuilder = new StringBuilder();
            foreach (var t in data)
            {
                sBuilder.Append(t.ToString("x2"));
            }
            return sBuilder.ToString().ToUpperInvariant();
        }

        public static string GetRandomMd5Hash()
        {
            var bytes = RandomNumberGenerator.GetBytes(16);
            return BitConverter.ToString(bytes).Replace("-", string.Empty).ToLower();
        }

        public static string GetApplicationPath()
        {
            return AppDomain.CurrentDomain.BaseDirectory;
        }

        public static string GetRelativePath(string relativeTo, string path)
        {
            var uriRelativeTo = new Uri(relativeTo);
            var rel = Uri.UnescapeDataString(uriRelativeTo.MakeRelativeUri(new Uri(path)).ToString())
                .Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar);
            if (!rel.Contains(Path.DirectorySeparatorChar.ToString()))
            {
                rel = $".{Path.DirectorySeparatorChar}{rel}";
            }
            return rel;
        }

        public static int GetNthIndexOf(string s, char c, int instance)
        {
            if (string.IsNullOrEmpty(s)) return -1;
            var count = 0;
            for (var i = 0; i < s.Length; i++)
            {
                if (s[i] == c)
                {
                    count++;
                    if (count == instance) return i;
                }
            }
            return -1;
        }
    }
}
