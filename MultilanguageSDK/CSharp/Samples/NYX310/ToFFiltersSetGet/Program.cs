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
        static int Main(string[] args)
        {
            Console.WriteLine("---ToFFiltersSetGet---");

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

            //The parameters of TimeFilter and ConfidenceFilter are stored in camera
            //The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK
            Console.WriteLine();
            Console.WriteLine("---1. Test TimeFilter---");

            ScTimeFilterParams TimeFilterParams = new ScTimeFilterParams();
            TimeFilterParams.threshold = 1;
            TimeFilterParams.enable = 0;
            status = VNAPI.VN_GetTimeFilterParams(deviceHandle, ref TimeFilterParams);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetTimeFilterParams] success status:(" + status + "). The default time filter switch is " + (TimeFilterParams.enable == 1 ? "true" : "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetTimeFilterParams] fail status:(" + status + ").");
                return 1;
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
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetTimeFilterParams] success status:(" + status + "). Set time filter switch to  " + (TimeFilterParams.enable == 1 ? "true" : "false") + " is Ok.");
            }
            else
            {
                Console.WriteLine("[VN_SetTimeFilterParams] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine();
            Console.WriteLine("---2. Test ConfidenceFilter---");

            ScConfidenceFilterParams confidenceFilterParams = new ScConfidenceFilterParams();
            confidenceFilterParams.enable = 1;
            confidenceFilterParams.threshold = 15;
            status = VNAPI.VN_GetConfidenceFilterParams(deviceHandle, ref confidenceFilterParams);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetConfidenceFilterParams] success status:(" + status + "). The default confidence filter switch is " + (confidenceFilterParams.enable == 1 ? "true" : "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetConfidenceFilterParams] fail status:(" + status + ").");
                return 1;
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
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetConfidenceFilterParams] success status:(" + status + "). Set confidence filter switch to  " + (confidenceFilterParams.enable == 1 ? "true" : "false") + " is Ok.");
            }
            else
            {
                Console.WriteLine("[VN_SetConfidenceFilterParams] fail status:(" + status + ").");
                return 1;
            }

            Console.WriteLine();
            Console.WriteLine("---3. Test FlyingPixelFilter---");

            ScFlyingPixelFilterParams flyingPixelFilterParams = new ScFlyingPixelFilterParams();
            flyingPixelFilterParams.enable = 1;
            flyingPixelFilterParams.threshold = 15;
            status = VNAPI.VN_GetFlyingPixelFilterParams(deviceHandle, ref flyingPixelFilterParams);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetFlyingPixelFilterParams] success status:(" + status + "). The default flying pixel filter switch is " + (flyingPixelFilterParams.enable == 1 ? "true" : "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetFlyingPixelFilterParams] fail status:(" + status + ").");
                return 1;
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
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetFlyingPixelFilterParams] success status:(" + status + "). Set flying pixel filter switch to  " + (flyingPixelFilterParams.enable == 1 ? "true" : "false") + " is Ok.");
            }
            else
            {
                Console.WriteLine("[VN_SetFlyingPixelFilterParams] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine();
            Console.WriteLine("---4. Test FillHoleFilter---");

            byte bFillHoleFilter = new byte();
            status = VNAPI.VN_GetFillHoleFilterEnabled(deviceHandle, ref bFillHoleFilter);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetFillHoleFilterEnabled] success status:(" + status + "). The default fill hole filter switch is " + (bFillHoleFilter == 1 ? "true" : "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetFillHoleFilterEnabled] fail status:(" + status + ").");
                return 1;
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
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetFillHoleFilterEnabled] success status:(" + status + "). Set fill hole filter switch to  " + (bFillHoleFilter == 1 ? "true" : "false") + " is Ok.");
            }
            else
            {
                Console.WriteLine("[VN_SetFillHoleFilterEnabled] fail status:(" + status + ").");
                return 1;
            }
            Console.WriteLine();
            Console.WriteLine("---5. Test SpatialFilter---");

            byte bSpatialFilter = new byte();
            status = VNAPI.VN_GetSpatialFilterEnabled(deviceHandle, ref bSpatialFilter);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetSpatialFilterEnabled] success status:(" + status + "). The default spatial filter switch is " + (bSpatialFilter == 1 ? "true" : "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetSpatialFilterEnabled] fail status:(" + status + ").");
                return 1;
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
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetSpatialFilterEnabled] success status:(" + status + "). Set spatial filter switch to  " + (bSpatialFilter == 1 ? "true" : "false") + " is Ok.");
            }
            else
            {
                Console.WriteLine("[VN_SetSpatialFilterEnabled] fail status:(" + status + ").");
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
