using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;

namespace ToFExposureTimeOfHDRSetGet
{
    using ScDeviceHandle = System.IntPtr;
    public struct ScExposureTimeParams
    {
        public ScExposureControlMode mode;
        public int exposureTime;
    };
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---ToFExposureTimeOfHDRSetGet---");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;
            
            //Initialize the ScepterSDK.
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

            //Get the count of devices.
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

            //Start the data stream.
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

            //Set frame rate.
            int frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetFrameRate] success status:(" + status + "). Get frameRate " + frameRate);
            }
            else
            {
                Console.WriteLine("[VN_SetFrameRate] fail status:(" + status + ").");
                return 1;
            }

            //Get HDR mode enabled status.
            byte enabled = 0;
            status = VNAPI.VN_GetHDRModeEnabled(deviceHandle, ref enabled);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetHDRModeEnabled] success status:(" + status + "). Get HDR mode status: " + (enabled == 1 ? "true": "false"));
            }
            else
            {
                Console.WriteLine("[VN_GetHDRModeEnabled] fail status:(" + status + ").");
                return 1;
            }

            //If HDR is disabled, enable it.
            if (0 == enabled)
            {
                //Get exposure control mode.
                ScExposureControlMode eControlMode = ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO;
                status = VNAPI.VN_GetExposureControlMode(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref eControlMode);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_GetExposureControlMode] success status:(" + status + "). Get exposure control mode: " + ((eControlMode == ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO) ? "SC_EXPOSURE_CONTROL_MODE_AUTO":"SC_EXPOSURE_CONTROL_MODE_MANUAL") );
                }
                else
                {
                    Console.WriteLine("[VN_GetExposureControlMode] fail status:(" + status + ").");
                    return 1;
                }

                //Set exposure control manual mode.
                if (ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO == eControlMode)
                {
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
                }

                //Set HDR mode enabled, WDR function need to be turned off in advance.
                status = VNAPI.VN_SetHDRModeEnabled(deviceHandle, 1);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_SetHDRModeEnabled] success status:(" + status + "). Enabled HDR mode.");
                }
                else
                {
                    Console.WriteLine("[VN_SetHDRModeEnabled] fail status:(" + status + ").");
                    return 1;
                }
            }

            //Get the frame counts, set the exposure time directly for individual frame count,
            //or set the exposure time separately for every frame count.
            int nCount = 0;
            status = VNAPI.VN_GetFrameCountOfHDRMode(deviceHandle, ref nCount);
            if (ScStatus.SC_OK == status)
            {
                Console.WriteLine("[VN_GetFrameCountOfHDRMode] success status:(" + status + "). Get HDR mode fame count: " + nCount);
                for (int i = 0; i < nCount; i++)
                {
                    //Get max exposure time of the specified frame count.
                    int maxExposureTime = 0;
                    status = VNAPI.VN_GetMaxExposureTimeOfHDR(deviceHandle, (byte)i, ref maxExposureTime);
                    if (status == ScStatus.SC_OK)
                    {
                        Console.WriteLine("[VN_GetMaxExposureTimeOfHDR] success status:(" + status + "). Get frame count: " + i + " max HDR exposure time: " + maxExposureTime);
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetMaxExposureTimeOfHDR] fail status:(" + status + ").");
                        return 1;
                    }

                    //Get current exposure time of the specified frame count.
                    int curExposureTime = 0;
                    status = VNAPI.VN_GetExposureTimeOfHDR(deviceHandle, (byte)i, ref curExposureTime);
                    if (status == ScStatus.SC_OK)
                    {
                        Console.WriteLine("[VN_GetExposureTimeOfHDR] success status:(" + status + "). Get frame count: " + i + " current HDR exposure time: " + curExposureTime);
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetExposureTimeOfHDR] fail status:(" + status + ").");
                        return 1;
                    }

                    //Set exposure time of the specified frame count.
                    int exposureTime = maxExposureTime / 2;
                    status = VNAPI.VN_SetExposureTimeOfHDR(deviceHandle, (byte)i, exposureTime);
                    if (status == ScStatus.SC_OK)
                    {
                        Console.WriteLine("[VN_SetExposureTimeOfHDR] success status:(" + status + "). Set frame count: " + i + " HDR exposure time: " + exposureTime);
                    }
                    else
                    {
                        Console.WriteLine("[VN_SetExposureTimeOfHDR] fail status:(" + status + ").");
                        return 1;
                    }
                }
            }
            else
            {
                Console.WriteLine("[VN_GetFrameCountOfHDRMode] fail status:(" + status + ").");
                return 1;
            }

            //Stop the started stream.
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
