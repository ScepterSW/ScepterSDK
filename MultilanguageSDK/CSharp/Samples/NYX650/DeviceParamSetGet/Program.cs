using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace DeviceParamSetGet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceParamSetGet---");

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

            //cameraParameters
            ScSensorIntrinsicParameters cameraParameters = new ScSensorIntrinsicParameters();
            status = VNAPI.VN_GetSensorIntrinsicParameters(deviceHandle, ScSensorType.SC_TOF_SENSOR, ref cameraParameters);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetCameraParameters failed status:" + status);
                return;
            }
            Console.WriteLine("Get ScGetCameraParameters status: " + status);
            Console.WriteLine("Depth Camera Intinsic:");
            Console.WriteLine("Fx: " + cameraParameters.fx);
            Console.WriteLine("Cx: " + cameraParameters.cx);
            Console.WriteLine("Fy: " + cameraParameters.fy);
            Console.WriteLine("Cy: " + cameraParameters.cy);

            Console.WriteLine("Depth Distortion Coefficient: ");
            Console.WriteLine("K1: " + cameraParameters.k1);
            Console.WriteLine("K2: " + cameraParameters.k2);
            Console.WriteLine("P1: " + cameraParameters.p1);
            Console.WriteLine("P2: " + cameraParameters.p2);
            Console.WriteLine("K3: " + cameraParameters.k3);
            Console.WriteLine("K4: " + cameraParameters.k4);
            Console.WriteLine("K5: " + cameraParameters.k5);
            Console.WriteLine("K6: " + cameraParameters.k6);

            //gmmgain
            byte gmmgain = 0;
            status = VNAPI.VN_GetIRGMMGain(deviceHandle, ref gmmgain);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetGMMGain failed status:" + status);
                return;
            }
            Console.WriteLine("default gmmgain: " + (int)gmmgain);

            gmmgain = 50;
            status = VNAPI.VN_SetIRGMMGain(deviceHandle, gmmgain);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetGMMGain failed status:" + status);
                return;
            }
            status = VNAPI.VN_GetIRGMMGain(deviceHandle, ref gmmgain);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetGMMGain failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set gmmgain: " + (int)gmmgain + " succeeded");
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

            Console.WriteLine("--end--");
            return ;
        }
    }
}
