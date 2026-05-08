using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;

namespace AIContinuousRunMode
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---AIContinuousRunMode---");

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
            //Set bSetParams to false. if the params has been initialized by "Parameter initialization file" in ScepterGUITool or by SDK API in "xxxx" sample.
            bool bSetParams = true;
            if (bSetParams)
            {
                status = SetParams(deviceHandle, ref VNAPI);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[SetParams] success status:(" + status + ").");
                }
                else
                {
                    Console.WriteLine("[SetParams] fail status:(" + status + ").");
                    return 1;
                }
            }
            UInt32 paramID = 0;
            IntPtr pBuffer = new IntPtr();
            ushort nBufferSize = 0;
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status == ScStatus.SC_OK)
            {
                string strRes = Marshal.PtrToStringAnsi(pBuffer, nBufferSize);
                Console.WriteLine("[VN_AIModuleGetParam] success status:(" + status + "). The algo's version is: " + strRes);
            }
            else
            {
                Console.WriteLine("[VN_AIModuleGetParam] fail status:(" + status + ").");
                return 1;
            }
            paramID = 1;
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status == ScStatus.SC_OK)
            {
                string strRes = Marshal.PtrToStringAnsi(pBuffer, nBufferSize);
                Console.WriteLine("[VN_AIModuleGetParam] success status:(" + status + "). The algo's name is: " + strRes);
            }
            else
            {
                Console.WriteLine("[VN_AIModuleGetParam] fail status:(" + status + ").");
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

            //Get the result form AI module.
            for (int j = 0; j < 100; j++)
            {
                ScAIResult aiResult = new ScAIResult();
                status = VNAPI.VN_AIModuleGetResult(deviceHandle, 1200, ref aiResult);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_AIModuleGetResult failed status:" + status);
                    Thread.Sleep(5);
                    continue;
                }
                Console.WriteLine("VN_AIModuleGetResult resultIndex :" + aiResult.resultIndex);
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

        public static ScStatus SetParams(ScDeviceHandle deviceHandle, ref ScepterAPI VNAPI)
        {
            ScStatus status = ScStatus.SC_OTHERS;
            //Set AI module continuous running.
            status = VNAPI.VN_AIModuleSetWorkMode(deviceHandle, ScAIModuleMode.AI_CONTINUOUS_RUN_MODE);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_AIModuleSetWorkMode] success status:(" + status + "). Set AI_CONTINUOUS_RUN_MODE.");
            }
            else
            {
                Console.WriteLine("[VN_AIModuleSetWorkMode] fail status:(" + status + ").");
                return status;
            }


            //Enable AI module.
            status = VNAPI.VN_AIModuleSetEnabled(deviceHandle, 1);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_AIModuleSetEnabled] success status:(" + status + "). Enable AI module.");
            }
            else
            {
                Console.WriteLine("[VN_AIModuleSetEnabled] fail status:(" + status + ").");
                return status;
            }

            return ScStatus.SC_OK;
        }
    }
}
