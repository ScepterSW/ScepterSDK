using System;
using System.Text;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;

namespace ColorExposureTimeSetGet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---ColorExposureTimeSetGet---");

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
            //Get default frame rate
            Int32 rate = 10;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref rate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetFrameRate] success status:(" + status + "). The device frameRate is " + rate + "\n");
            }
            else
            {
                Console.WriteLine("[VN_GetFrameRate] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine("---- To  SC_EXPOSURE_CONTROL_MODE_MANUAL ----");
            //switch exposure mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureControlMode] success status:(" + status + "). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success.");
            }
            else
            {
                Console.WriteLine("[VN_SetExposureControlMode] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine("---1. Get color sensor exposure time range with frame rate " + rate + "---");
            //Get the range of the Auto Color exposure time 
            Int32 maxExposureTime = 10;
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref maxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetMaxExposureTime] success status:(" + status + "). Recommended scope: 100 - " + maxExposureTime);
            }
            else
            {
                Console.WriteLine("[VN_GetMaxExposureTime] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine("---2. Set and get color sensor new exposure time---");
            //Set new ExposureTime
            Int32 exposureTime = 3000;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, exposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureTime] success status:(" + status + ").Set the device exposure time to " + exposureTime);
            }
            else
            {
                Console.WriteLine("[VN_SetExposureTime] fail status:(" + status + ").");
                return 1;
            }

            status = VNAPI.VN_GetExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref exposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetExposureTime] success status:(" + status + "). The device exposure time is " + exposureTime + "\n");
            }
            else
            {
                Console.WriteLine("[VN_GetExposureTime] fail status:(" + status + ").");
                return 1;
            }

            float colorGain = 3.5F;
            status = VNAPI.VN_SetColorGain(deviceHandle,  colorGain);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetColorGain] success status:(" + status + "). Set the device color gain to " + colorGain);
            }
            else
            {
                Console.WriteLine("[VN_SetColorGain] fail status:(" + status + ").");
                return 1;
            }
            colorGain = 0.0F;
            status = VNAPI.VN_GetColorGain(deviceHandle, ref colorGain);

            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetColorGain] success status:(" + status + "). The device color gain is " + colorGain + "\n");
            }
            else
            {
                Console.WriteLine("[VN_GetColorGain] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine("---- To SC_EXPOSURE_CONTROL_MODE_AUTO ----");
            //switch exposure mode to auto
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetExposureControlMode] success status:(" + status + "). Set SC_EXPOSURE_CONTROL_MODE_AUTO success.");
            }
            else
            {
                Console.WriteLine("[VN_SetExposureControlMode] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine("1. Get color exposure time range---");
            //Get the range of the Auto Color exposure time 
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref maxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetMaxExposureTime] success status:(" + status + "). Recommended scope: 100 - " + maxExposureTime);
            }
            else
            {
                Console.WriteLine("[VN_GetMaxExposureTime] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine("---2. Set and get color sensor new max exposure time range in auto mode---");
            //set new range of Auto Color exposure time. [100  maxExposureTime]
            Int32 AECMaxExposureTime = 3000;
            status = VNAPI.VN_SetColorAECMaxExposureTime(deviceHandle, AECMaxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetColorAECMaxExposureTime] success status:(" + status + "). Set color AEC max exposure time to  " + AECMaxExposureTime);
            }
            else
            {
                Console.WriteLine("[VN_SetColorAECMaxExposureTime] fail status:(" + status + ").");
                return 1;
            }

            //Get the new range of the Auto Color exposure time .
            status = VNAPI.VN_GetColorAECMaxExposureTime(deviceHandle, ref AECMaxExposureTime);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetColorAECMaxExposureTime] success status:(" + status + "). Get color AEC max exposure time to  " + AECMaxExposureTime + "\n");
            }
            else
            {
                Console.WriteLine("[VN_GetColorAECMaxExposureTime] fail status:(" + status + ").");
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
