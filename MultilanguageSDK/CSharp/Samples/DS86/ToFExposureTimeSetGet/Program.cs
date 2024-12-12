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
        static void Main(string[] args)
        {
            Console.WriteLine("---- ToFExposureTimeSetGet ----");

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
        //2.Get infomation of the devices. 
        //3.Open devices accroding to the info.
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

            //Testing TOF exposure time requires turning off HDR in advance.
            //1.Default FrameRate
            //2.Set new ExposureTime
            //3.Change FrameRate to 5 (The exposure time ranges are different at different frame rates)
            //4.Set new ExposureTime

            Console.WriteLine("---- Default FrameRate ----");
            //Set Control mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_TOF_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetExposureControlMode failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set control mode to manual.");
            }

            //Get default frame rate
            int defaultframeRate = new int();
            defaultframeRate = 10;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref defaultframeRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("Get default frame rate: " + defaultframeRate);

            //Get the range of the ToF exposure time 
            Int32 maxExposureTime = 10;
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref maxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetMaxExposureTime failed Status::" + status);
                return;
            }
            Console.WriteLine("Recommended scope: 58 - " + maxExposureTime);

            //Set new ExposureTime
            Int32 exposureTime = 400;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, exposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set exposure time " + exposureTime + " is OK.");
            }

            Console.WriteLine("---- Set FrameRate to 5 ----");
            //Set new FrameRate
            int frameRate = new int();  //New frame rate, can change it
            frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFrameRate set new FrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("Set frame rate " + frameRate + " is OK.");

            //Need to get new ExposureTime Range due to FrameRate change.
            status = VNAPI.VN_GetMaxExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref maxExposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetMaxExposureTime failed Status::" + status);
                return;
            }
            Console.WriteLine("Recommended scope: 58 - " + maxExposureTime);

            //Set new ExposureTime 500
            exposureTime = 500;
            status = VNAPI.VN_SetExposureTime(deviceHandle, ScSensorType.SC_TOF_SENSOR, exposureTime);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set exposure time " + exposureTime + " is OK.");
            }

            //Stop capturing the image stream
            status = VNAPI.VN_StopStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StopStream failed status:" + status);
                return;
            }

            //1.Close device
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

            Console.WriteLine("--- Test end, please reboot camera to restore the default settings ---");
            return ;
        }
    }
}
