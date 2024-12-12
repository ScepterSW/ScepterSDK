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
        static void Main(string[] args)
        {
            Console.WriteLine("---MultiConnection---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
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
            do
            {
                status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("scGetDeviceCount failed! make sure the NYX is connected");
                    Console.ReadKey(true);
                    return;
                }
                Console.WriteLine("Get device count: " + deviceCount);
            } while (deviceCount < 2);
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
            for (int i = 0; i < deviceCount; i++)
            {
                Console.WriteLine("CameraIndex: " + i);
                Console.WriteLine("serialNumber:" + pDeviceListInfo[i].serialNumber);
                Console.WriteLine("ip:" + pDeviceListInfo[i].ip);
                Console.WriteLine("connectStatus:" + pDeviceListInfo[i].status);
            }
            ScDeviceHandle[] deviceHandle = new IntPtr[deviceCount];
            for (int i = 0; i < deviceCount; i++)
            {
                status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[i].serialNumber, ref deviceHandle[i]);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("OpenDevice failed status:" + status);
                    return;
                }
            }

            for (int i = 0; i < deviceCount; i++)
            {
                //Starts capturing the image stream
                status = VNAPI.VN_StartStream(deviceHandle[i]);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_StartStream failed status:" + status);
                    return;
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
                            Console.WriteLine(pDeviceListInfo[i].serialNumber + "  VN_GetFrameReady failed status:" + status);
                            continue;
                        }

                        //Get depth frame, depth frame only output in following data mode
                        if (1 == frameReady.depth)
                        {
                            status = VNAPI.VN_GetFrame(deviceHandle[i], ScFrameType.SC_DEPTH_FRAME, ref depthFrame);

                            if (status == ScStatus.SC_OK && depthFrame.pFrameData != IntPtr.Zero)
                            {
                                Console.WriteLine(pDeviceListInfo[i].serialNumber + " frameIndex :" + depthFrame.frameIndex);
                            }
                            else
                            {
                                Console.WriteLine(pDeviceListInfo[i].serialNumber + "VN_GetFrame SC_DEPTH_FRAME status:" + status);
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
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_StopStream failed status:" + status);
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle[i]);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_CloseDevice failed status:" + status);
                }
            }
            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }

            Console.WriteLine("--end--");

            return;
        }
    }
}
