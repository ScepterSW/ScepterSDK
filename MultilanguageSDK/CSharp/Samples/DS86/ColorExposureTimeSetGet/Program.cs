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
        static void Main(string[] args)
        {
            Console.WriteLine("---ColorExposureTimeSetGet---");

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

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }
            //Get default frame rate
            Int32 rate = 10;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref rate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("---- To  SC_EXPOSURE_CONTROL_MODE_MANUAL ----");
            //switch exposure mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetExposureControlMode failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("VN_SetExposureControlMode  ok");
            }
            Console.WriteLine("* step1. Get Color exposure time range with frameRate " + rate + "*");

            //Get the range of the Auto Color exposure time 
            Int32 maxExposureTime = 10;
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref maxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetMaxExposureTime failed Status::" + status);
                return;
            }
            Console.WriteLine("Recommended scope: 100 - " + maxExposureTime);

            Console.WriteLine("* step2. Set and Get new ExposureTime *");
            //Set new ExposureTime
            Int32 exposureTime = 3000;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, exposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("SetExposureTime:" + exposureTime);
            }

            status = VNAPI.VN_GetExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref exposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("GetExposureTime:" + exposureTime);
            }
            Console.WriteLine("* Set and Get ColorGain *");

            //set new ColorGain
            float colorGain = new float();
            colorGain = 3.5f;
            status = VNAPI.VN_SetColorGain(deviceHandle, colorGain);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorGain failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("SetColorGain:" + colorGain);
            }

            status = VNAPI.VN_GetColorGain(deviceHandle,ref colorGain);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetColorGain failed status:" + status);
                return ;
            }
            else
            {
                Console.WriteLine("GetColorGain:" + colorGain);
            }

            Console.WriteLine("---- To SC_EXPOSURE_CONTROL_MODE_AUTO ----");
            //switch exposure mode to auto
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("Sc_SetExposureControlMode failed status:" + status);
                return ;
            }
            else
            {
                Console.WriteLine("Sc_SetExposureControlMode ok");
            }

            Console.WriteLine("* step1. Get Color exposure time range *");
            //Get the range of the Auto Color exposure time 
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref maxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetMaxExposureTime failed Status::" + status);
                return;
            }
            Console.WriteLine("Recommended scope: 100 - " + maxExposureTime);

            Console.WriteLine("* step2. Set and Get new Auto Max Color exposure time range *");
            //set new range of Auto Color exposure time. [100  maxExposureTime]
            Int32 AECMaxExposureTime = 10000;
            status = VNAPI.VN_SetColorAECMaxExposureTime(deviceHandle, AECMaxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorAECMaxExposureTime failed status:" + status);
                return ;
            }
            else
            {
                Console.WriteLine("VN_SetColorAECMaxExposureTime:" + AECMaxExposureTime);
            }

            //Get the new range of the Auto Color exposure time .
            status = VNAPI.VN_GetColorAECMaxExposureTime(deviceHandle, ref AECMaxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetColorAECMaxExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("VN_GetColorAECMaxExposureTime:" + AECMaxExposureTime);
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

            return ;
        }
    }
}
