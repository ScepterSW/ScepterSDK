using System;
using System.Threading;
using System.Runtime.InteropServices;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace MultiConnection
{
    using ScDeviceHandle = System.IntPtr;

    class Program
    {
        //static ScDeviceInfo[] pDeviceListInfo;
        static int Main(string[] args)
        {
            Console.WriteLine("---MultiConnection---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
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
            do
            {
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
            } while (deviceCount < 2);

            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetDeviceInfoList] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceInfoList] fail status:" + status);
                return 1;
            }
            for (int i = 0; i < deviceCount; i++)
            {
                Console.WriteLine("The device index: " + i + ", <serialNumber>: " + pDeviceListInfo[i].serialNumber + ", <ip>: " + pDeviceListInfo[i].ip + ", <status>: " + pDeviceListInfo[i].status);
            }
            ScDeviceHandle[] deviceHandle = new IntPtr[deviceCount];
            for (int i = 0; i < deviceCount; i++)
            {
                status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[i].serialNumber, ref deviceHandle[i]);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_OpenDeviceBySN] success status:" + status + " The device index: " + i + ", <serialNumber>: " + pDeviceListInfo[i].serialNumber);
                }
                else
                {
                    Console.WriteLine("[VN_OpenDeviceBySN] fail status:" + status + " The device index: " + i + ", <serialNumber>: " + pDeviceListInfo[i].serialNumber);
                    return 1;
                }
            }

            for (int i = 0; i < deviceCount; i++)
            {
                //Starts capturing the image stream
                status = VNAPI.VN_StartStream(deviceHandle[i]);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_StartStream] success status:" + status + " The device index: " + i );
                }
                else
                {
                    Console.WriteLine("[VN_StartStream] fail status:" + status + " The device index: " + i );
                    return 1;
                }
            }

            //Wait for the device to upload image data
            Thread.Sleep(1000);

            //1.ReadNextFrame.
            //2.Get depth Frame acoording to Ready flag.
            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < deviceCount; i++)
                {
                    if (IntPtr.Zero != deviceHandle[i])
                    {
                        ScFrame depthFrame = new ScFrame();
                        ScFrameReady frameReady = new ScFrameReady();
                        status = VNAPI.VN_GetFrameReady(deviceHandle[i], 1200, ref frameReady);

                        if (status != ScStatus.SC_OK)
                        {
                            Console.WriteLine("[VN_GetFrameReady] fail status:" + status + " The device index: " + i );
                            continue;
                        }

                        //Get depth frame, depth frame only output in following data mode
                        if (1 == frameReady.depth)
                        {
                            status = VNAPI.VN_GetFrame(deviceHandle[i], ScFrameType.SC_DEPTH_FRAME, ref depthFrame);

                            if (status == ScStatus.SC_OK && depthFrame.pFrameData != IntPtr.Zero)
                            {
                                Console.WriteLine("[VN_GetFrame] success status:" + status + " The device index: " + i + ", SC_DEPTH_FRAME <frameIndex>: " + depthFrame.frameIndex);
                            }
                            else
                            {
                                Console.WriteLine("[VN_GetFrame] fail status:" + status + " The device index: " + i );
                            }
                        }
                    }

                }
            }

            //1.close device
            //2.SDK shutdown
            for (int i = 0; i < deviceCount; i++)
            {
                status = VNAPI.VN_StopStream(deviceHandle[i]);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_StopStream] success status:" + status + " The device index: " + i );
                }
                else
                {
                    Console.WriteLine("[VN_StopStream] fail status:" + status + " The device index: " + i );
                    return 1;
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle[i]);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_CloseDevice] success status:" + status + " The device index: " + i );
                }
                else
                {
                    Console.WriteLine("[VN_CloseDevice] fail status:" + status + " The device index: " + i );
                    return 1;
                }
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
