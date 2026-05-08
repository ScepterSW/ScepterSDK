using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace DeviceSetFrameRate
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---DeviceSetFrameRate---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //SDK Initialize
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

            //1.Search and notice the count of devices.
            //2.get infomation of the devices.
            //3.open devices accroding to the info.
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

            int frameRate = new int();
            frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetFrameRate] success status:(" + status + "). Set the device frame rate to " + frameRate);
            }
            else
            {
                Console.WriteLine("[VN_SetFrameRate] fail status:(" + status + ").");
                return 1;
            }

            status = VNAPI.VN_SetWorkMode(deviceHandle, ScWorkMode.SC_ACTIVE_MODE);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetWorkMode] success status:(" + status + "). Set SC_ACTIVE_MODE.");
            }
            else
            {
                Console.WriteLine("[VN_SetWorkMode] fail status:(" + status + ").");
                return 1;
            }

            //Starts capturing the image stream
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

            Console.WriteLine("Start testing the average frame rate for 30 seconds, Please wait patiently");
            //statistical frame rate

            const int TESTPERIOD = 30;//30 senconds
            int index = 0;
            UInt64 start = 0;
            start = GetTimeStampMS();

            for (; ; )
            {
                ScFrameReady FrameReady = new ScFrameReady();
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_GetFrameReady] success status:(" + status + ").");
                }
                else
                {
                    Console.WriteLine("[VN_GetFrameReady] fail status:(" + status + ").");
                    continue;
                }

                if (1 == FrameReady.depth)
                {
                    ScFrame depthFrame = new ScFrame();
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref depthFrame);
                    if (status == ScStatus.SC_OK)
                    {
                        Console.WriteLine("[VN_GetFrame] success status:(" + status + "). SC_DEPTH_FRAME <frameIndex>: " + depthFrame.frameIndex);
                        UInt64 diff = GetTimeStampMS() - start;
                        index++;
                        if (diff > (TESTPERIOD * 1000))
                        {
                            float fps = (index * TESTPERIOD * 1000.0f / diff) / TESTPERIOD;
                            index = 0;
                            Console.WriteLine("Average frame rate: " + fps);
                            break;
                        }
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetFrame] fail status:(" + status + ").");
                    }
                }

            }

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

        static UInt64 GetTimeStampMS()
        {
            TimeSpan ts = DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, 0);
            UInt64 ret = Convert.ToUInt64(ts.TotalMilliseconds);
            return ret;
        }
    }
}
