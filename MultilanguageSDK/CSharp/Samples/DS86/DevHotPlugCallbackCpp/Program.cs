using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace DevHotPlugCallbackCpp
{
    using ScDeviceHandle = System.IntPtr;
    class Sensor
    {
        public static ScepterAPI VNAPI = new ScepterAPI();
        public static ScDeviceHandle deviceHandle = new IntPtr();

        public static void HotPlugStateCallback(ref ScDeviceInfo pInfo, int status, IntPtr contex)
        {
            HandleCallback(ref pInfo, status);
        }

        public ScStatus registCallback(ScDeviceHandle deviceHand)
        {
            deviceHandle = deviceHand;
            IntPtr pt = IntPtr.Zero;

            ScepterAPI.PtrHotPlugStatusCallback hpcb = new ScepterAPI.PtrHotPlugStatusCallback(HotPlugStateCallback);
            return VNAPI.VN_SetHotPlugStatusCallback(hpcb, pt);
        }

        public static void HandleCallback(ref ScDeviceInfo pInfo, int status)
        {
            Console.WriteLine("ip " + status + "  " + pInfo.ip + "    " + (status == 0 ? "add" : "remove"));
            Console.WriteLine("serialNumber " + status + "  " + pInfo.serialNumber + "    " + (status == 0 ? "add" : "remove"));

            if (status == 0)
            {
                Console.WriteLine("VN_OpenDevice " + VNAPI.VN_OpenDeviceBySN(pInfo.serialNumber, ref deviceHandle));
                Console.WriteLine("VN_StartStream " + VNAPI.VN_StartStream(deviceHandle));
            }
            else
            {
                Console.WriteLine("VN_StopStream " + VNAPI.VN_StopStream(deviceHandle));
                Console.WriteLine("VN_CloseDevice " + VNAPI.VN_CloseDevice(ref deviceHandle));
            }

        }
    }

    class Program
    {
        public static ScDeviceHandle deviceHandle = new IntPtr();
        public static ScepterAPI VNAPI = new ScepterAPI();

        static void Main(string[] args)
        {
            UInt32 deviceCount = 0;

            ScStatus status = VNAPI.VN_Initialize();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Initialize failed status:" + status);
                Console.ReadKey(true);
                return;
            }

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

            if (InitDevice(deviceCount))
            {
                Sensor s  = new Sensor();
                status = (ScStatus)s.registCallback(deviceHandle);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("SetHotPlugStatusCallback failed status:" + status);
                }
                else
                {
                    Console.WriteLine(" wait for hotplug operation ");
                    // wait for hotplug
                    for (; ; )
                    {
                        Thread.Sleep(1000);
                    }
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("CloseDevice failed status:" + status);
                }
            }
            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("Shutdown failed status:" + status);
            }
        }

        public static bool InitDevice(UInt32 deviceCount)
        {
            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            ScStatus status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status" + pDeviceListInfo[0].status);
                    Console.WriteLine("The device state does not support connection." );
                    return false;
                }
            }
            else
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return false;
            }

            Console.WriteLine("serialNumber:" + pDeviceListInfo[0].serialNumber);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            deviceHandle = IntPtr.Zero;

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return false;
            }

            Console.WriteLine("VN_OpenDeviceBySN,status :" + status);

            status = VNAPI.VN_StartStream(deviceHandle);

            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("StartStream failed status:" + status);
                return false;
            }

            return true;
        }
    }
}
