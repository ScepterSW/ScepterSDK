using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace IRGMMCorrectionSetGet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("--------------IRGMMCorrectionSetGet-------------");

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

            //The parameters of IRGMMCorrection are stored in camera

            Console.WriteLine("-------------------------- test IRGMMCorrection --------------------------");

            ScIRGMMCorrectionParams IRGMMCorrectionParams = new ScIRGMMCorrectionParams();
            IRGMMCorrectionParams.threshold = 25;
            IRGMMCorrectionParams.enable = 1;
            status = VNAPI.VN_GetIRGMMCorrection(deviceHandle, ref IRGMMCorrectionParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetIRGMMCorrection failed status:" + status);
                return;
            }

            if(IRGMMCorrectionParams.enable == 0)
            {
                Console.WriteLine("The default IRGMMCorrection switch is False");
                IRGMMCorrectionParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default IRGMMCorrection switch is True");
                IRGMMCorrectionParams.enable = 0;
            }
            
            status = VNAPI.VN_SetIRGMMCorrection(deviceHandle, IRGMMCorrectionParams);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetIRGMMCorrection failed status:" + status);
                return;
            }
            Console.WriteLine("Set IRGMMCorrection switch to " + (IRGMMCorrectionParams.enable == 1 ? "true" : "false") + " is Ok.");

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scStartStream failed status:" + status);
                return;
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

            Console.WriteLine("---Test end, please reboot camera to restore the default settings.----");
            return;
        }
    }
}
