using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;

namespace ToFExposureTimeSetGet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---ToFExposureTimeSetGet---");

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
        //2.Get infomation of the devices. 
        //3.Open devices accroding to the info.
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

            //Testing TOF exposure time requires turning off HDR and WDR in advance.
            //1.Default FrameRate
            //2.Set new ExposureTime
            //3.Change FrameRate to 5 (The exposure time ranges are different at different frame rates)
            //4.Set new ExposureTime
            Console.WriteLine();
            Console.WriteLine("---- 1.Default FrameRate ----");
            //Set Control mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_TOF_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureControlMode] success status:(" + status + "). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success.");
            }
            else
            {
                Console.WriteLine("[VN_SetExposureControlMode] fail status:(" + status + ").");
                return 1;
            }

            //Get default frame rate
            int defaultframeRate = new int();
            defaultframeRate = 10;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref defaultframeRate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetFrameRate] success status:(" + status + "). The device frame rate is " + defaultframeRate);
            }
            else
            {
                Console.WriteLine("[VN_GetFrameRate] fail status:(" + status + ").");
                return 1;
            }

            //Get the range of the ToF exposure time 
            Int32 maxExposureTime = 10;
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref maxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetMaxExposureTime] success status:(" + status + "). Recommended scope: 58 - " + maxExposureTime);
            }
            else
            {
                Console.WriteLine("[VN_GetMaxExposureTime] fail status:(" + status + ").");
                return 1;
            }

            //Set new ExposureTime
            Int32 exposureTime = 400;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, exposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureTime] success status:(" + status + "). Set exposure time " + exposureTime + " is OK.");
            }
            else
            {
                Console.WriteLine("[VN_SetExposureTime] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine();
            Console.WriteLine("---- 2.Set FrameRate to 5 ----");
            //Set new FrameRate
            int frameRate = new int();  //New frame rate, can change it
            frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetFrameRate] success status:(" + status + "). Sey frame rate is " + frameRate + " is OK.");
            }
            else
            {
                Console.WriteLine("[VN_SetFrameRate] fail status:(" + status + ").");
                return 1;
            }

            //Need to get new ExposureTime Range due to FrameRate change.
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref maxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetMaxExposureTime] success status:(" + status + "). Recommended scope: 58 - " + maxExposureTime);
            }
            else
            {
                Console.WriteLine("[VN_GetMaxExposureTime] fail status:(" + status + ").");
                return 1;
            }

            //Set new ExposureTime 500
            exposureTime = 500;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, exposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureTime] success status:(" + status + "). Set exposure time " + exposureTime + " is OK.");
            }
            else
            {
                Console.WriteLine("[VN_SetExposureTime] fail status:(" + status + ").");
                return 1;
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
