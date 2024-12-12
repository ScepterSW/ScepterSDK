﻿using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace DeviceUpgradeFirmWare
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceUpgradeFirmWare---");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status;

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

            //Input Json file path

            Console.WriteLine("Please input firmware file path:");
            string pImgPath = Console.ReadLine();

            //Start upgrade.
            status = VNAPI.VN_StartUpgradeFirmWare(deviceHandle, pImgPath);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scStartUpgradeFirmWare failed status:" + status);
                return;
            }

            int upgradeStatus = 0;
            int process = 0;
            while (true)
            {
                //Get the current upgrade status.
                status = VNAPI.VN_GetUpgradeStatus(deviceHandle, ref upgradeStatus, ref process);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("scGetUpgradeStatus failed status:" + status);
                    break;
                }
                else
                {
                    Console.WriteLine("Upgrade firmWare status:" + upgradeStatus + ", process:" + process);
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
                            Console.WriteLine("Upgrade OK.");
                            break;
                        }
                    }
                }
                Thread.Sleep(1000);
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
