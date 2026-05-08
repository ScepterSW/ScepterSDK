using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;
using System.Collections.Generic;
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct AlgHeader
{
    /*
	//Start identifer:STX A L G
	pBuf[0] = 0x02; //STX
	pBuf[1] = 0x41; //A
	pBuf[2] = 0x4C; //L
	pBuf[3] = 0x47; //G
	uint32_t startIdentifer = 0x474C4102;
	*/
    public uint startIdentifer;
    public ushort sizeOfValidData;
    public byte checksum; 
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct DetectedResult
{
    public byte index;      
    public byte safetyLevel;
    public byte isWarning;
}

namespace SafetyCameraGetResult
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        public static int ParseAIResult(ScAIResult result, List<DetectedResult> detectedResults)
        {
            try
            {
                uint usedLen = 0;

                // 1. check data length
                if (result.dataLen < Marshal.SizeOf(typeof(AlgHeader)))
                {
                    Console.WriteLine($"[ERROR] DataLen is invalid. Insufficient data length: {result.dataLen} bytes (header requires {Marshal.SizeOf(typeof(AlgHeader))} bytes)");
                    return -1;
                }

                // 2. parse header
                    AlgHeader header = Marshal.PtrToStructure<AlgHeader>(result.pResultData);
                if (header.startIdentifer != 0x474C4102) // "GL\x01" in little-endian
                {
                    Console.WriteLine($"[ERROR] startIdentifer is invalid: 0x{header.startIdentifer:X8} (expected 0x474C4102)");
                    return -1;
                }
                usedLen += (uint)Marshal.SizeOf(typeof(AlgHeader));

                // 3. check data vaild length
                if (result.dataLen < usedLen + header.sizeOfValidData)
                {
                    Console.WriteLine($"[ERROR] Invalid data length: {header.sizeOfValidData} bytes specified but only {result.dataLen - usedLen} bytes available");
                    return -1;
                }

                // 4. check checksum
                byte calculatedChecksum = 0;

                for (int i = 0; i < header.sizeOfValidData; i++)
                {
                    byte temp = Marshal.ReadByte(result.pResultData, (int)usedLen + i);
                    calculatedChecksum += temp;
                }

                if (calculatedChecksum != header.checksum)
                {
                    Console.WriteLine($"[ERROR] Checksum mismatch: calculated 0x{calculatedChecksum:X2}, expected 0x{header.checksum:X2}");
                    return -1;
                }

                // 5. parse PackageID (1 byte)
                byte packageID = Marshal.ReadByte(result.pResultData, (int)usedLen);
                usedLen += 1;
                if (packageID != 0x00)
                {
                    Console.WriteLine($"[ERROR] Invalid package ID: 0x{packageID:X2} (expected 0x00)");
                    return -1;
                }

                // 6. parse result length (2 bytes)
                ushort resultLen = (ushort)Marshal.ReadInt16(result.pResultData, (int)usedLen);
                usedLen += 2;

                // 7.parse state and error code(2 bytes)
                byte state = Marshal.ReadByte(result.pResultData, (int)usedLen);
                usedLen += 1;
                byte errorCode = Marshal.ReadByte(result.pResultData, (int)usedLen);
                usedLen += 1;

                if (state != 0 || errorCode != 0)
                {
                    Console.WriteLine($"[ERROR] State check failed - state: 0x{state:X2}, errorCode: 0x{errorCode:X2}");
                    return -1;
                }

                // 8. parse detection area number(1 byte)
                byte areaCount = Marshal.ReadByte(result.pResultData, (int)usedLen);
                usedLen += 1;

                // 9. parse detection result(3 bytes for one result)
                    int resultSize = Marshal.SizeOf(typeof(DetectedResult));
                for (int i = 0; i < areaCount; i++)
                {
                    if (usedLen + resultSize > result.dataLen)
                    {
                        Console.WriteLine($"[WARNING] Truncated data at result {i + 1}/{areaCount}");
                        break;
                    }

                    DetectedResult dr = Marshal.PtrToStructure<DetectedResult>(
                        IntPtr.Add(result.pResultData, (int)usedLen));
                    detectedResults.Add(dr);
                    usedLen += (uint)resultSize;
                }

                // 10. record the rest data
                if (result.dataLen > usedLen)
                {
                    uint remaining = result.dataLen - usedLen;
                    Console.WriteLine($"[INFO] Result {result.resultIndex} has {remaining} bytes remaining data.");
                }
                return 0;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CRITICAL] Parsing failed: {ex.Message}");
                return -1;
            }
        }
        static int Main(string[] args)
        {
            Console.WriteLine("---SafetyCameraGetResult---");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //Initialize the ScepterSDK.
            status = VNAPI.VN_Initialize();
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_Initialize] success ScStatus:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_Initialize] fail ScStatus:(" + status + ").");
                Console.ReadKey(true);
                return 1;
            }

            //Get the count of devices.
            status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetDeviceCount] success ScStatus:(" + status + "). The device count is " + deviceCount);
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceCount] fail ScStatus:(" + status + ").");
                Console.ReadKey(true);
                return 1;
            }
            if (0 == deviceCount)
            {
                Console.WriteLine("[VN_GetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.");
                Console.ReadKey(true);
                return 1;
            }

            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetDeviceInfoList] success status:(" + status + ")." + "The first deviceInfo, <serialNumber>:" + pDeviceListInfo[0].serialNumber + ", <ip>:" + pDeviceListInfo[0].ip + ", <status>:" + pDeviceListInfo[0].status);
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status" + pDeviceListInfo[0].status + "The device state does not support connection.");
                    return 1;
                }
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceInfoList] fail status:" + status);
                return 1;
            }

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_OpenDeviceBySN] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_OpenDeviceBySN] fail status:(" + status + ").");
                return 1;
            }

            //Start the data stream.
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_StartStream] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_StartStream] fail status:(" + status + ").");
                return 1;
            }

            //Get the result form AI module.
            for (int j = 0; j < 100; j++)
            {
                ScAIResult aiResult = new ScAIResult();
                status = VNAPI.VN_AIModuleGetResult(deviceHandle, 1200, ref aiResult);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_AIModuleGetResult] fail status:(" + status + ").");
                    Thread.Sleep(5);
                    continue;
                }
                List<DetectedResult> detectedResults = new List<DetectedResult>();
                ParseAIResult(aiResult, detectedResults);
                for(int ind = 0; ind < detectedResults.Count; ind ++)
                {
                    if (detectedResults[ind].isWarning == 1)
                    {
                        Console.WriteLine("area " + detectedResults[ind].index + " is dangerous.");
                    }
                }
            }

            //Stop capturing the image stream
            status = VNAPI.VN_StopStream(deviceHandle);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_StopStream] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_StopStream] fail status:(" + status + ").");
                return 1;
            }

            //1.close device
            //2.SDK shutdown
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_CloseDevice] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_CloseDevice] fail status:(" + status + ").");
                return 1;
            }
            status = VNAPI.VN_Shutdown();
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_Shutdown] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_Shutdown] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine("---End---");

            return 0;
        }
    }
}
