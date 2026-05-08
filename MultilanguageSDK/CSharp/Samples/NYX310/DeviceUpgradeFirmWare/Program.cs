using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.IO;

namespace DeviceUpgradeFirmWare
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---DeviceUpgradeFirmWare---");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status;

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
            }
            else
            {
                Console.WriteLine("[VN_GetDeviceInfoList] fail ScStatus:(" + status + ").");
                Console.ReadKey(true);
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

            //Input Json file path
            string pImgPath = @"./camera.fw";
            if (File.Exists(pImgPath) == false)
            {
                Console.WriteLine("Please input firmware file path:");
                pImgPath = Console.ReadLine();
            }

            //Start upgrade.
            status = VNAPI.VN_StartUpgradeFirmWare(deviceHandle, pImgPath);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_StartUpgradeFirmWare] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_StartUpgradeFirmWare] fail status:(" + status + ").");
                return 1;
            }

            int upgradeStatus = 0;
            int process = 0;
            while (true)
            {
                //Get the current upgrade status.
                status = VNAPI.VN_GetUpgradeStatus(deviceHandle, ref upgradeStatus, ref process);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_GetUpgradeStatus] fail status:(" + status + ").");
                    break;
                }
                else
                {
                    Console.WriteLine("[VN_GetUpgradeStatus] success status:(" + status + "). " + "Upgrade firmWare status:" + upgradeStatus + ", process:" + process);
                    if (upgradeStatus != 0)
                    {
                        Console.WriteLine("upgrade failed.");
                        break;
                    }
                    else
                    {
                        //Upgrade progress is 100, upgrade successful. After the upgrade is successful, 
                        //the SDK will automatically reboot the device internally to make the upgrade file effective.
                        if (process == 100)
                        {
                            Console.WriteLine("Upgrade Done.");
                            Console.WriteLine("Waiting for reboot.");
                            Thread.Sleep(10000);
                            Console.WriteLine("Reboot done.");
                            break;
                        }
                    }
                }
                Thread.Sleep(1000);
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
