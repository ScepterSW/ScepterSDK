using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace ToFFiltersSetGet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("--------------ToFFiltersSetGet-------------");

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

            //The parameters of TimeFilter and ConfidenceFilter are stored in camera
            //The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK

            Console.WriteLine("-------------1------------ test TimeFilter --------------------------");

            ScTimeFilterParams TimeFilterParams = new ScTimeFilterParams();
            TimeFilterParams.threshold = 1;
            TimeFilterParams.enable = 0;
            status = VNAPI.VN_GetTimeFilterParams(deviceHandle, ref TimeFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetTimeFilterParams failed status:" + status);
                return;
            }

            if(TimeFilterParams.enable == 0)
            {
                Console.WriteLine("The default TimeFilter switch is False");
                TimeFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default TimeFilter switch is True" );
                TimeFilterParams.enable = 0;
            }
            
            status = VNAPI.VN_SetTimeFilterParams(deviceHandle, TimeFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetTimeFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set TimeFilter switch to " + (TimeFilterParams.enable == 1 ? "true" : "false") + " is Ok.");

            Console.WriteLine("-------------2--------- test ConfidenceFilter -----------------------");

            ScConfidenceFilterParams confidenceFilterParams = new ScConfidenceFilterParams();
            confidenceFilterParams.enable = 1;
            confidenceFilterParams.threshold = 15;
            status = VNAPI.VN_GetConfidenceFilterParams(deviceHandle, ref confidenceFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetConfidenceFilterParams failed status:" + status);
                return;
            }
            if (confidenceFilterParams.enable == 0)
            {
                Console.WriteLine("The default ConfidenceFilter switch is False");
                confidenceFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default ConfidenceFilter switch is True");
                confidenceFilterParams.enable = 0;
            }
            
            status = VNAPI.VN_SetConfidenceFilterParams(deviceHandle, confidenceFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetConfidenceFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set ConfidenceFilter switch to " + (confidenceFilterParams.enable == 1 ? "true" : "false") + " is Ok.");


            Console.WriteLine("-------------3--------- test FlyingPixelFilter ----------------------");

            ScFlyingPixelFilterParams flyingPixelFilterParams = new ScFlyingPixelFilterParams();
            flyingPixelFilterParams.enable = 1;
            flyingPixelFilterParams.threshold = 15;
            status = VNAPI.VN_GetFlyingPixelFilterParams(deviceHandle, ref flyingPixelFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetFlyingPixelFilterParams failed status:" + status);
                return;
            }

            if (flyingPixelFilterParams.enable == 0)
            {
                Console.WriteLine("The default flyingPixelFilterParams switch is False");
                flyingPixelFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default flyingPixelFilterParams switch is True");
                flyingPixelFilterParams.enable = 0;
            }

            status = VNAPI.VN_SetFlyingPixelFilterParams(deviceHandle, flyingPixelFilterParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFlyingPixelFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set FlyingPixelFilter switch to " + (flyingPixelFilterParams.enable == 1 ? "true" : "false") + " is Ok.");

            Console.WriteLine("-------------4---------- test FillHoleFilter ------------------------");

            byte bFillHoleFilter = new byte();
            status = VNAPI.VN_GetFillHoleFilterEnabled(deviceHandle, ref bFillHoleFilter);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetFillHoleFilterEnabled failed status:" + status);
                return;
            }

            if (bFillHoleFilter == 0)
            {
                Console.WriteLine("The default FillHoleFilter switch is False");
                bFillHoleFilter = 1;
            }
            else
            {
                Console.WriteLine("The default FillHoleFilter switch is True");
                bFillHoleFilter = 0;
            }
            
            status = VNAPI.VN_SetFillHoleFilterEnabled(deviceHandle, bFillHoleFilter);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFillHoleFilterEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Set FillHoleFilter switch to " + (bFillHoleFilter == 1 ? "true" : "false") + " is Ok.");

            Console.WriteLine("-------------5---------- test SpatialFilter -------------------------");

            byte bSpatialFilter = new byte();
            status = VNAPI.VN_GetSpatialFilterEnabled(deviceHandle, ref bSpatialFilter);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetSpatialFilterEnabled failed status:" + status);
                return;
            }
            if (bSpatialFilter == 0)
            {
                Console.WriteLine("The default SpatialFilter switch is False");
                bSpatialFilter = 1;
            }
            else
            {
                Console.WriteLine("The default SpatialFilter switch is True");
                bSpatialFilter = 0;

            }
            status = VNAPI.VN_SetSpatialFilterEnabled(deviceHandle, bSpatialFilter);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetSpatialFilterEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Set SpatialFilter switch to " + (bSpatialFilter == 1 ? "true":"false") + " is Ok.");

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

            Console.WriteLine("---Test end, please reboot camera to restore the default settings.----");
            return;
        }
    }
}
