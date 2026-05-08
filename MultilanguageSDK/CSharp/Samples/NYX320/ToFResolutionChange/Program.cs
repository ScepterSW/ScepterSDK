using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace ToFResolutionChange
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---ToFResolutionChange---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;

            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about frame

            ScFrameReady FrameReady = new ScFrameReady();
            ScFrame TofFrame = new ScFrame();

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
                Console.WriteLine("[VN_GetDeviceCount] success ScStatus:(" + status + ").");
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

            //switch TofResolution
            int resolution_w = new int();
            resolution_w =  640;
            int resolution_h = new int();
            resolution_h = 480;
            Console.WriteLine();
            Console.WriteLine("---Test ToFResolution 640 * 480 ---");
            //1.640_480
            status = VNAPI.VN_SetToFResolution(deviceHandle, resolution_w, resolution_h);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetToFResolution] success status:(" + status + "). Set tof sensor resolution 640 * 480.");
            }
            else
            {
                Console.WriteLine("[VN_SetToFResolution] fail status:(" + status + ").");
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

            for (int i = 0; i < 10; i++)
            {
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

                //cout resolution
                if (1 == FrameReady.depth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref TofFrame);
                    if (status == ScStatus.SC_OK && TofFrame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("[VN_GetFrame] success status:(" + status + "). SC_DEPTH_FRAME <frameIndex>: " + TofFrame.frameIndex + " resolution: " + TofFrame.width + "*" + TofFrame.height);
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetFrame] fail status:(" + status + ").");
                    }
                }

            }

            resolution_w = 320;
            resolution_h = 240;
            Console.WriteLine();
            Console.WriteLine("---Test TofResolution 320 * 240 ---");
            //2.320_240
            status = VNAPI.VN_SetToFResolution(deviceHandle, resolution_w, resolution_h);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetToFResolution] success status:(" + status + "). Set tof sensor resolution 320 * 240.");
            }
            else
            {
                Console.WriteLine("[VN_SetToFResolution] fail status:(" + status + ").");
                return 1;
            }

            for (int i = 0; i < 10; i++)
            {
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

                //cout resolution
                if (1 == FrameReady.depth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref TofFrame);
                    if (status == ScStatus.SC_OK && TofFrame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("[VN_GetFrame] success status:(" + status + "). SC_DEPTH_FRAME <frameIndex>: " + TofFrame.frameIndex + " resolution: " + TofFrame.width + "*" + TofFrame.height);
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetFrame] fail status:(" + status + ").");
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
