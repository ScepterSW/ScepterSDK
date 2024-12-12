using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace ColorResolutionChange
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---ColorResolutionChange---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;

            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about frame

            ScFrameReady FrameReady = new ScFrameReady();
            ScFrame ColorFrame = new ScFrame();

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
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo); ;
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

            Console.WriteLine("serialNumber:" + pDeviceListInfo[0].serialNumber);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }

            Console.WriteLine("VN_OpenDeviceBySN,status :" + status);

            //switch ColorResolution
            int resolution_w = new int();
            resolution_w =  640;
            int resolution_h = new int();
            resolution_h = 480;
            //1.640_480
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 640_480");
            }

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            for (int i = 0; i < 10; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                //cout resolution
                if (1 == FrameReady.color)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_COLOR_FRAME, ref ColorFrame);
                    if (status == ScStatus.SC_OK && ColorFrame.pFrameData != IntPtr.Zero
                    && ColorFrame.width == 640)
                    {
                        Console.WriteLine("VN_GetFrame,status:" + status + "  "
                             + "resolution: " + ColorFrame.width + "x" + ColorFrame.height);
                    }
                }

            }

            resolution_w = 1600;
            resolution_h = 1200;
            //2.1600_1200
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 1600_1200");
            }

            for (int i = 0; i < 10; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                //cout resolution
                if (1 == FrameReady.color)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_COLOR_FRAME, ref ColorFrame);
                    if (status == ScStatus.SC_OK && ColorFrame.pFrameData != IntPtr.Zero
                                && ColorFrame.width == 1600)

                    {
                        Console.WriteLine("VN_GetFrame,status:" + status + "  "
                            + "resolution: " + ColorFrame.width + "x" + ColorFrame.height);
                    }
                }

            }
            //Stop capturing the image stream
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

            return;
        }
    }
}
