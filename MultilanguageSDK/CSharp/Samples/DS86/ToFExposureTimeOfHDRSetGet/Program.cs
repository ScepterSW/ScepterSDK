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
        static void Main(string[] args)
        {
            Console.WriteLine("---- ToFExposureTimeOfHDRSetGet ----");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;
            
            //Initialize the ScepterSDK.
            status = VNAPI.VN_Initialize();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scInitialize failed status:" + status);
                Console.ReadKey(true);
                return;
            }

            //Get the count of devices.
            status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scGetDeviceCount failed! make sure pointer valid or called scInitialize()");
                Console.ReadKey(true);
                return;
            }
            Console.WriteLine("Get device count: " + deviceCount);
            if (0 == deviceCount)
            {
                Console.WriteLine("scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.");
                Console.ReadKey(true);
                return;
            }

            //Get the infomation of devices.
            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status: " + pDeviceListInfo[0].status);
                    Console.WriteLine("The device state does not support connection.");
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

            //Open the first device by serial number.
            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }
            Console.WriteLine("scOpenDeviceBySN status :" + status);

            //Start the data stream.
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scStartStream failed status:" + status);
                return;
            }

            //Set frame rate.
            int frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scSetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("Set frame rate:" + frameRate);

            //Get HDR mode enabled status.
            byte enabled = 0;
            status = VNAPI.VN_GetHDRModeEnabled(deviceHandle, ref enabled);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scGetHDRModeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Get HDRMode status:" + enabled);

            //If HDR is disabled, enable it.
            if (0 == enabled)
            {
                //Get exposure control mode.
                ScExposureControlMode eControlMode = ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO;
                status = VNAPI.VN_GetExposureControlMode(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref eControlMode);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("scGetExposureControlMode failed status:" + status);
                    return;
                }
                Console.WriteLine("Get exposure control mode:" + eControlMode);

                //Set exposure control manual mode.
                if (ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO == eControlMode)
                {
                    status = VNAPI.VN_SetExposureControlMode(deviceHandle, ScSensorType.SC_TOF_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL);
                    if (status != ScStatus.SC_OK)
                    {
                        Console.WriteLine("scSetExposureControlMode failed status:" + status);
                        return;
                    }
                    Console.WriteLine("Set exposure control manual mode");
                }

                //Set HDR mode enabled.
                status = VNAPI.VN_SetHDRModeEnabled(deviceHandle, 1);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("scSetHDRModeEnabled failed status:" + status);
                    return;
                }
                Console.WriteLine("Open HDR mode");
            }

            //Get the frame counts, set the exposure time directly for individual frame count,
            //or set the exposure time separately for every frame count.
            int nCount = 0;
            status = VNAPI.VN_GetFrameCountOfHDRMode(deviceHandle, ref nCount);
            if (ScStatus.SC_OK == status)
            {
                for (int i = 0; i < nCount; i++)
                {
                    //Get max exposure time of the specified frame count.
                    int maxExposureTime = 0;
                    status = VNAPI.VN_GetMaxExposureTimeOfHDR(deviceHandle, (byte)i, ref maxExposureTime);
                    if (status != ScStatus.SC_OK)
                    {
                        Console.WriteLine("scGetMaxExposureTimeOfHDR FrameCount: " + i + " failed status:" + status);
                        return;
                    }
                    Console.WriteLine("Get FrameCount: " + i + " max exposure time:" + maxExposureTime);

                    //Get current exposure time of the specified frame count.
                    int curExposureTime = 0;
                    status = VNAPI.VN_GetExposureTimeOfHDR(deviceHandle, (byte)i, ref curExposureTime);
                    if (status != ScStatus.SC_OK)
                    {
                        Console.WriteLine("scGetExposureTimeOfHDR FrameCount: " + i + " failed status:" + status);
                        return;
                    }
                    Console.WriteLine("Get FrameCount: " + i + " current exposure time:" + curExposureTime);

                    //Set exposure time of the specified frame count.
                    int exposureTime = maxExposureTime / 2;
                    status = VNAPI.VN_SetExposureTimeOfHDR(deviceHandle, (byte)i, exposureTime);
                    if (status != ScStatus.SC_OK)
                    {
                        Console.WriteLine("scSetExposureTimeOfHDR FrameCount: " + i + " failed status:" + status);
                        return;
                    }
                    Console.WriteLine("Set FrameCount: " + i + " exposure time:" + exposureTime);
                }
            }
            else
            {
                Console.WriteLine("scGetFrameCountOfHDRMode failed status:" + status);
                return;
            }

            //Stop the started stream.
            status = VNAPI.VN_StopStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scStopStream failed status:" + status);
                return;
            }

            //Close the opened device.
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scCloseDevice failed status:" + status);
                return;
            }

            //Shutdown the initialized ScepterSDK.
            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scShutdown failed status:" + status);
                return;
            }

            Console.WriteLine("--- Test end---");
            return;
        }
    }
}
