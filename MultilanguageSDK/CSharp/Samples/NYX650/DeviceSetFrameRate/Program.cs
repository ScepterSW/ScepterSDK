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
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceSetFrameRate---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //SDK Initialize
            status = VNAPI.VN_Initialize();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Initialize failed status:" + status);
                Console.ReadKey(true);
                return;
            }

            //1.Search and notice the count of devices.
            //2.get infomation of the devices.
            //3.open devices accroding to the info.
            status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetDeviceCount failed! make sure pointer valid or called VN_Initialize");
                Console.ReadKey(true);
                return;
            }
            Console.WriteLine("Get device count: " + deviceCount);
            if (0 == deviceCount)
            {
                Console.WriteLine("VN_GetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.");
                Console.ReadKey(true);
                return;
            }

            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];

            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status" + pDeviceListInfo[0].status);
                    Console.WriteLine("The device state does not support connection." );
                    return;
                }
            }
            else
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return;
            }

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }
            Console.WriteLine("VN_OpenDeviceBySN,status :" + status);

            int frameRate = new int();
            frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("set frame rate :" + frameRate + " is OK.");
            status = VNAPI.VN_SetWorkMode(deviceHandle, ScWorkMode.SC_ACTIVE_MODE);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetWorkMode failed status:" + status);
                return;
            }
            Console.WriteLine("set active mode");
            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
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
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                if (1 == FrameReady.depth)
                {
                    ScFrame depthFrame = new ScFrame();
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref depthFrame);
                    if (depthFrame.pFrameData != IntPtr.Zero)
                    {
                        UInt64 diff = GetTimeStampMS() - start;
                        index++;
                        if (diff > (TESTPERIOD * 1000))
                        {
                            float fps = (index * TESTPERIOD * 1000.0f / diff) / TESTPERIOD;
                            index = 0;
                            Console.WriteLine(fps);
                            break;
                        }
                    }
                }

            }

            status = VNAPI.VN_StopStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StopStream failed status:" + status);
                return;
            }
            //1.close device
            //2.SDK shutdown
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_CloseDevice failed status:" + status);
                return;
            }
            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }
            Console.WriteLine("---end---");

            return ;
        }

        static UInt64 GetTimeStampMS()
        {
            TimeSpan ts = DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, 0);
            UInt64 ret = Convert.ToUInt64(ts.TotalMilliseconds);
            return ret;
        }
    }
}
