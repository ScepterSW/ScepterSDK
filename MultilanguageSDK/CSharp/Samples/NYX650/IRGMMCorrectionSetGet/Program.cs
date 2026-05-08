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
        static int Main(string[] args)
        {
            Console.WriteLine("---IRGMMCorrectionSetGet---");

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

            //The parameters of IRGMMCorrection are stored in camera

            ScIRGMMCorrectionParams IRGMMCorrectionParams = new ScIRGMMCorrectionParams();
            IRGMMCorrectionParams.threshold = 25;
            IRGMMCorrectionParams.enable = 1;
            status = VNAPI.VN_GetIRGMMCorrection(deviceHandle, ref IRGMMCorrectionParams);
            if (status == ScStatus.SC_OK)
            {
                if(IRGMMCorrectionParams.enable == 1)
                {
                    Console.WriteLine("[VN_GetIRGMMCorrection] success status:(" + status + "). The device default IR GMM correction enable is " + (IRGMMCorrectionParams.enable == 1 ? "true" : "false") + " , threshold is " + IRGMMCorrectionParams.threshold);
                }
                else
                {
                    Console.WriteLine("[VN_GetIRGMMCorrection] success status:(" + status + "). The device default IR GMM correction enable is " + (IRGMMCorrectionParams.enable == 1 ? "true" : "false"));
                }
            }
            else
            {
                Console.WriteLine("[VN_GetIRGMMCorrection] fail status:(" + status + ").");
                return 1;
            }

            if(IRGMMCorrectionParams.enable == 0)
            {
                IRGMMCorrectionParams.enable = 1;
                IRGMMCorrectionParams.threshold = IRGMMCorrectionParams.threshold / 2 + 30;
            }
            else
            {
                IRGMMCorrectionParams.enable = 0;
            }
            
            status = VNAPI.VN_SetIRGMMCorrection(deviceHandle, IRGMMCorrectionParams);
            if (status == ScStatus.SC_OK)
            {
                if(IRGMMCorrectionParams.enable == 1)
                {
                    Console.WriteLine("[VN_GetIRGMMCorrection] success status:(" + status + "). Set the device IR GMM correction enable is " + (IRGMMCorrectionParams.enable == 1 ? "true" : "false") + " , threshold is " + IRGMMCorrectionParams.threshold);
                }
                else
                {
                    Console.WriteLine("[VN_GetIRGMMCorrection] success status:(" + status + "). Set the device IR GMM correction enable is " + (IRGMMCorrectionParams.enable == 1 ? "true" : "false"));
                }
            }
            else
            {
                Console.WriteLine("[VN_GetIRGMMCorrection] fail status:(" + status + ").");
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
