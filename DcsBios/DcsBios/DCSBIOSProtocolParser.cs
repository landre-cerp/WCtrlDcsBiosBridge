
// ReSharper disable All
/*
 * Do not adhere to naming standard in DCS-BIOS code, standard are based on DCS-BIOS json files and byte streamnaming
 */

using System.Diagnostics;

namespace DCS_BIOS
{
    using System;
    using System.Collections.Concurrent;
    using System.Collections.Generic;
    using System.Linq;
    using System.Threading;

    using DCS_BIOS.EventArgs;
    using DCS_BIOS.misc;
    using NLog;

    public enum DCSBiosStateEnum
    {
        WAIT_FOR_SYNC = 0,
        ADDRESS_LOW = 1,
        ADDRESS_HIGH = 2,
        COUNT_LOW = 3,
        COUNT_HIGH = 4,
        DATA_LOW = 5,
        DATA_HIGH = 6,
    }

    /// <summary>
    /// Reads and interprets the data stream sent from DCS-BIOS.
    /// When a DCS-BIOS Control value has been received it
    /// checks whether anyone is listening for the specific Address
    /// and if so then broadcast the information.
    /// </summary>
    internal class DCSBIOSProtocolParser : IDisposable
    {
        internal static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        private readonly List<string> _errorsLogged = new(10);

        private DCSBiosStateEnum _state;
        private uint _address;
        private uint _count;
        private uint _data;
        private byte _syncByteCount;
        private bool _shutdown;
        private static readonly object _lockListOfAddressesToBroascastObject = new();
        private readonly List<uint> _listOfAddressesToBroascast = new();
        public static DCSBIOSProtocolParser DCSBIOSProtocolParserSO;
        private AutoResetEvent _autoResetEvent = new(false);

        private readonly ConcurrentQueue<byte[]> _arrayQueue = new();
        private Thread _processingThread;

        private DCSBIOSProtocolParser()
        {
            _state = DCSBiosStateEnum.WAIT_FOR_SYNC;
            _syncByteCount = 0;
            DCSBIOSProtocolParserSO = this;
            _shutdown = false;

            // Add addresses that are specifix to the send cycle
            _listOfAddressesToBroascast.Add(DCSBIOSConstants.META_MODULE_START_ADDRESS); // MetadataStart
            _listOfAddressesToBroascast.Add(DCSBIOSConstants.META_MODULE_END_ADDRESS); // MetadataEnd
        }

        protected virtual void Dispose(bool disposing)
        {
            _shutdown = true;
            if (disposing)
            {
                // dispose managed resources
                _autoResetEvent?.Set();
                _autoResetEvent?.Dispose();
                _autoResetEvent = null;
            }
            // free native resources
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        public void Startup()
        {
            _shutdown = false;
            _autoResetEvent = new (false);
            _processingThread = new Thread(ProcessArraysThread);
            _processingThread.Start();
        }

        public void Shutdown()
        {
            _shutdown = true;
            _autoResetEvent?.Set();
            _autoResetEvent?.Dispose();
            _autoResetEvent = null;
        }

        public void AddArray(byte[] bytes)
        {
            if (_shutdown == true) return;

            _arrayQueue.Enqueue(bytes);
            _autoResetEvent.Set();
        }

        private void ProcessArraysThread()
        {
            try
            {
                while (!_shutdown)
                {
                    try
                    {
                        byte[] array = null;
                        
                        while (_arrayQueue.TryDequeue(out array))
                        {
                            if (array != null)
                            {
                                for (int i = 0; i < array.Length; i++)
                                {
                                    ProcessByte(array[i]);
                                }
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Logger.Error(ex, $"DCSBIOSProtocolParser.ProcessArrays(), arrays to process : {_arrayQueue.Count}");
                    }

                    _autoResetEvent.WaitOne();
                }
            }
            catch (Exception ex)
            {
                Logger.Error(ex, $"DCSBIOSProtocolParser.ProcessArrays(), arrays to process : {_arrayQueue.Count}");
            }
        }

        public static void RegisterAddressToBroadCast(uint address)
        {
            GetParser();
            lock (_lockListOfAddressesToBroascastObject)
            {
                if (!DCSBIOSProtocolParserSO._listOfAddressesToBroascast.Any(u => u == address))
                {
                    DCSBIOSProtocolParserSO._listOfAddressesToBroascast.Add(address);
                }
            }
        }

        public static DCSBIOSProtocolParser GetParser()
        {
            if (DCSBIOSProtocolParserSO == null)
            {
                DCSBIOSProtocolParserSO = new DCSBIOSProtocolParser();
            }
            return DCSBIOSProtocolParserSO;
        }

        private bool IsBroadcastable(uint address)
        {
            var result = false;
            lock (_lockListOfAddressesToBroascastObject)
            {
                if (_listOfAddressesToBroascast.Any(u => u == address))
                {
                    result = true;
                }
            }
            return result;
        }

        internal void ProcessByte(byte b)
        {
            try
            {
                switch (_state)
                {
                    case DCSBiosStateEnum.WAIT_FOR_SYNC:
                        /* do nothing */
                        break;
                    case DCSBiosStateEnum.ADDRESS_LOW:
                        _address = b;
                        _state = DCSBiosStateEnum.ADDRESS_HIGH;
                        break;
                    case DCSBiosStateEnum.ADDRESS_HIGH:
                        _address = (uint)(b << 8) | _address;
                        _state = _address != 0x5555 ? DCSBiosStateEnum.COUNT_LOW : DCSBiosStateEnum.WAIT_FOR_SYNC;
                        break;
                    case DCSBiosStateEnum.COUNT_LOW:
                        _count = b;
                        _state = DCSBiosStateEnum.COUNT_HIGH;
                        break;
                    case DCSBiosStateEnum.COUNT_HIGH:
                        _count = (uint)(b << 8) | _count;
                        _state = DCSBiosStateEnum.DATA_LOW;
                        break;
                    case DCSBiosStateEnum.DATA_LOW:
                        _data = b;
                        _count--;
                        _state = DCSBiosStateEnum.DATA_HIGH;
                        break;
                    case DCSBiosStateEnum.DATA_HIGH:
                        _data = (uint)(b << 8) | _data;
                        _count--;


                        if (IsBroadcastable(_address))
                        {
                            try
                            {
                                BIOSEventHandler.DCSBIOSDataAvailable(this, _address, _data);
                            }
                            catch (Exception ex)
                            {
                                if (!_errorsLogged.Contains(ex.Message))
                                {
                                    Logger.Error(ex, "Error in DCS-BIOS stream. This error will be logged *just once*.");
                                    _errorsLogged.Add(ex.Message);
                                }
                            }
                        }

                        _address += 2;
                        if (_count == 0)
                            _state = DCSBiosStateEnum.ADDRESS_LOW;
                        else
                            _state = DCSBiosStateEnum.DATA_LOW;
                        break;
                }

                if (b == 0x55)
                {
                    //Console.WriteLine(Environment.TickCount - ticks);
                    //ticks = Environment.TickCount;
                    _syncByteCount++;
                }
                else
                {
                    _syncByteCount = 0;
                }

                if (_syncByteCount == 4)
                {
                    _state = DCSBiosStateEnum.ADDRESS_LOW;
                    _syncByteCount = 0;
                }
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "DCSBIOSProtocolParser.ProcessByte()");
            }
        }
    }
}
