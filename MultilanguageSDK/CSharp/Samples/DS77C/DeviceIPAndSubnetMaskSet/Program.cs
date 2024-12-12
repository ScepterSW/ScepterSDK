using System;
using System.Text;
using System.Threading;
using System.Runtime.InteropServices;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace DeviceIPAndSubnetMaskSet
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceIPAndSubnetMaskSet---");

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

            //Set the device as non-DHCP mode.
            byte bEnabled = 0;
            status = VNAPI.VN_SetDeviceDHCPEnabled(deviceHandle, bEnabled);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("SetDeviceDHCPEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Set device DHCP disabled.");

            //Set the IP address of the device in non-DHCP mode.
            string strIP = "192.168.1.102";
            IntPtr pIP = Marshal.StringToHGlobalAnsi(strIP);
            status = VNAPI.VN_SetDeviceIPAddr(deviceHandle, pIP, 14);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("Set IP failed status:" + status);
                return;
            }
            Console.WriteLine("Set IP:" + strIP + " OK.");

            //Set the subnet mask of the device in non-DHCP mode.
            string strSubnetMask = "255.255.255.0";
            IntPtr pSubnetMask = Marshal.StringToHGlobalAnsi(strSubnetMask);
            status = VNAPI.VN_SetDeviceSubnetMask(deviceHandle, pSubnetMask, 14);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("Set subnetMask failed status:" + status);
                return;
            }
            Console.WriteLine("Set SubnetMask:255.255.255.0 OK.");

            //When the device is rebooted, the set IP and subnet mask take effect.
            status = VNAPI.VN_RebootDevie(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("scRebootDevie failed status:" + status);
                return;
            }

            //close device
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_CloseDevice failed status:" + status);
                return;
            }
            //SDK shutdown
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
