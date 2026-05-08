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
            if (status == 0)
            {
                Console.WriteLine("The device is <Added>, deviceInfo <serialNumber>:  " + pInfo.serialNumber + ", <ip>: " + pInfo.ip + ", <status>: " + pInfo.status);
                ScStatus ret = VNAPI.VN_OpenDeviceBySN(pInfo.serialNumber, ref deviceHandle);
                if (ret == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_OpenDeviceBySN] success status:(" + ret + ").");
                }
                else
                {
                    Console.WriteLine("[VN_OpenDeviceBySN] fail status:(" + ret + ").");
                    return;
                }
                ret =  VNAPI.VN_StartStream(deviceHandle);
                if (ret == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_StartStream] success status:(" + ret + ").");
                }
                else
                {
                    Console.WriteLine("[VN_StartStream] fail status:(" + ret + ").");
                    return;
                }
            }
            else
            {
                Console.WriteLine("The device is <Removed>, deviceInfo <serialNumber>:  " + pInfo.serialNumber + ", <ip>: " + pInfo.ip + ", <status>: " + pInfo.status);
                ScStatus ret = VNAPI.VN_StopStream(deviceHandle);
                if (ret == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_StopStream] success status:(" + ret + ").");
                }
                else
                {
                    Console.WriteLine("[VN_StopStream] fail status:(" + ret + ").");
                    return ;
                }
                ret =  VNAPI.VN_CloseDevice(ref deviceHandle);
                if (ret == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_CloseDevice] success status:(" + ret + ").");
                }
                else
                {
                    Console.WriteLine("[VN_CloseDevice] fail status:(" + ret + ").");
                    return;
                }
            }

        }
    }

    class Program
    {
        public static ScDeviceHandle deviceHandle = new IntPtr();
        public static ScepterAPI VNAPI = new ScepterAPI();

        static int Main(string[] args)
        {
            Console.WriteLine("---DevHotPlugCallbackCpp---");
            UInt32 deviceCount = 0;
            ScStatus status = VNAPI.VN_Initialize();
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_Initialize] success ScStatus:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_Initialize] failed ScStatus:(" + status + ").");
                Console.ReadKey(true);
                return 1;
            }

            status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetDeviceCount] success ScStatus:(" + status + "). The device count is " + deviceCount);
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceCount] failed ScStatus:(" + status + ").");
                Console.ReadKey(true);
                return 1;
            }

            if (0 == deviceCount)
            {
                Console.WriteLine("[VN_GetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.");
                Console.ReadKey(true);
                return 1;
            }

            if (InitDevice(deviceCount))
            {
                Sensor s  = new Sensor();
                status = (ScStatus)s.registCallback(deviceHandle);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[registCallback] success status:(" + status + "). Waiting for hotplug operation.");
                    // wait for hotplug
                    for (; ; )
                    {
                        Thread.Sleep(1000);
                    }
                }
                else
                {
                    Console.WriteLine("[registCallback] fail status:(" + status + ").");
                    //return 1;
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_CloseDevice] success status:(" + status + ").");
                }
                else
                {
                    Console.WriteLine("[VN_CloseDevice] failed status:(" + status + ").");
                    return 1;
                }
            }
            else
            {
                Console.WriteLine("InitDevice failed");
                return 1;
            }
            status = VNAPI.VN_Shutdown();
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_Shutdown] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_Shutdown] failed status:(" + status + ").");
                return 1;
            }
            return 0;
        }

        public static bool InitDevice(UInt32 deviceCount)
        {
            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            ScStatus status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetDeviceInfoList] success status:(" + status + ")." + "The first deviceInfo, <serialNumber>:" + pDeviceListInfo[0].serialNumber + ", <ip>:" + pDeviceListInfo[0].ip + ", <status>:" + pDeviceListInfo[0].status);
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status" + pDeviceListInfo[0].status + "The device state does not support connection.");
                    return false;
                }
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceInfoList] failed status:" + status);
                return false;
            }

            deviceHandle = IntPtr.Zero;

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_OpenDeviceBySN] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_OpenDeviceBySN] fail status:(" + status + ").");
                return false;
            }

            status = VNAPI.VN_StartStream(deviceHandle);

            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_StartStream] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_StartStream] fail status:(" + status + ").");
                return false;
            }

            return true;
        }
    }
}
