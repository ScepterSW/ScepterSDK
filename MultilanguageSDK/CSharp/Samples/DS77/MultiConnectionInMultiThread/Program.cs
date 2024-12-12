using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace MultiConnectionInMultiThread
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---MultiConnectionInMultiThread---");

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
                    Console.WriteLine("VN_GetDeviceCount failed! make sure pointer valid or called VN_Initialize()");
                    Console.ReadKey(true);
                    return;
                }
                Console.WriteLine("Get device count: " + deviceCount);
            } while (deviceCount < 2);

            ScDeviceHandle[] deviceHandle = new ScDeviceHandle[deviceCount];
            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];

            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return;
            }
            else
            {
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status" + pDeviceListInfo[0].status);
                    Console.WriteLine("The device state does not support connection." );
                    return;
                }
                Device[] cDevice = new Device[deviceCount];
                Thread[] t = new Thread[deviceCount];
                for (UInt32 i = 0; i < deviceCount; i++)
                {
                    cDevice[i] = new Device(pDeviceListInfo[i], ref VNAPI);
                    t[i] = new Thread(cDevice[i].TestDevice);
                    t[i].Start();
                }

                for (UInt32 i = 0; i < deviceCount; i++)
                {
                    cDevice[i].WaitForTestDone();
                    t[i].Abort();
                }
            }

            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }

            Console.WriteLine("--end--");

            return ;
        }
    }

    class Device
    {
        public Device(ScDeviceInfo DeviceListInfo, ref ScepterAPI pVNAPI)
        {
            pDeviceListInfo.ip = DeviceListInfo.ip;
            pDeviceListInfo.serialNumber = DeviceListInfo.serialNumber;
            pDeviceListInfo.status = DeviceListInfo.status;
            VNAPI = pVNAPI;
        }
        public void TestDevice()
        {
            Console.WriteLine("TestDevice");
            lock (this)
            {
                Console.WriteLine("serialNumber:" + pDeviceListInfo.serialNumber);
                Console.WriteLine("ip:" + pDeviceListInfo.ip);
                Console.WriteLine("connectStatus:" + pDeviceListInfo.status);

                ScStatus status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo.serialNumber, ref device_);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("OpenDevice " + pDeviceListInfo.serialNumber + " failed status:" + status);
                }

                // Starts capturing the image stream
                status = VNAPI.VN_StartStream(device_);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_StartStream " + pDeviceListInfo.serialNumber + " failed status:" + status);
                }

                //Wait for the device to upload image data
                Thread.Sleep(1000);

                // 1.ReadNextFrame.
                // 2.Get depth Frame acoording to Ready flag.
                for (int j = 0; j < 10; j++)
                {
                    ScFrame depthFrame = new ScFrame();
                    ScFrameReady frameReady = new ScFrameReady();
                    status = VNAPI.VN_GetFrameReady(device_, 1200, ref frameReady);

                    if (status != ScStatus.SC_OK)
                    {
                        Console.WriteLine(pDeviceListInfo.serialNumber + " VN_GetFrameReady failed status:" + status);
                        continue;
                    }

                    // Get depth frame, depth frame only output in following data mode
                    if (1 == frameReady.depth)
                    {
                        status = VNAPI.VN_GetFrame(device_, ScFrameType.SC_DEPTH_FRAME, ref depthFrame);

                        if (status == ScStatus.SC_OK && depthFrame.pFrameData != IntPtr.Zero)
                        {
                            Console.WriteLine(pDeviceListInfo.serialNumber + " frameIndex :" + depthFrame.frameIndex);
                        }
                        else
                        {
                            Console.WriteLine(pDeviceListInfo.serialNumber + "VN_GetFrame ScFrameType.SC_DEPTH_FRAME status:" + status);
                        }
                    }
                }

                // 1.close device
                // 2.SDK shutdown
                status = VNAPI.VN_StopStream(device_);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_StopStream failed status:" + status);
                }
                status = VNAPI.VN_CloseDevice(ref device_);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_CloseDevice failed status:" + status);
                }
                isTestDone_ = true;
            }
            return;
        }

        public bool WaitForTestDone()
        {
            while (!isTestDone_)
            {
                Thread.Sleep(1000);
            }
            return true;
        }
        
	    ScDeviceHandle device_;
        bool isTestDone_ = false;
        static ScepterAPI VNAPI;
        ScDeviceInfo pDeviceListInfo;
    }
}
