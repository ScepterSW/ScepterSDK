using System;
using System.IO;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace DeviceSWTriggerMode
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        public static int GetCurrentProcessId()
        {
            // Get PID
            return Process.GetCurrentProcess().Id;
        }

        public static double GetMemoryUsage(int pid)
        {
            // Get PID Mem(MB)
            var process = Process.GetProcessById(pid);
            var memoryInfo = process.PrivateMemorySize64; // RSS
            return memoryInfo / (1024 * 1024);
        }


        public static string GetInstanceNameByPid(int pid)
        {
            var processList = Process.GetProcesses();
            foreach (var process in processList)
            {
                if (process.Id == pid)
                {
                    return process.ProcessName;
                }
            }
            return null;
        }

        public static double GetCpuUsage(int pid)
        {
            // Get PID CPU occupy
            var name = GetInstanceNameByPid(pid);
            using (var counter = new PerformanceCounter("Process", "% Processor Time", name, true))
            {
                counter.NextValue(); // refrseh counter
                return (counter.NextValue()) / (Environment.ProcessorCount);
            }
        }

        static int Main(string[] args)
        {
            Console.WriteLine("---AIModelSingleResultTest---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about result
            ScAIResult pAIResult = new ScAIResult();
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
            // get frameRate
            int frameRate = 0;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref frameRate);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_GetFrameRate] success status:(" + status + "). Get frameRate " + frameRate);
            }
            else
            {
                Console.WriteLine("[VN_GetFrameRate] fail status:(" + status + ").");
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

            try
            {
                using (StreamWriter sWriter = new StreamWriter("AIModelSingleResultTest.csv"))
                {
                    sWriter.Close();
                }
            }
            catch (IOException ex)
            {
                Console.WriteLine("csv file open failed" + ex.Message);
                return 1;
            }

            StreamWriter csvWriter = new StreamWriter("AIModelSingleResultTest.csv");
            string strBuf = "resultIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure";
            csvWriter.WriteLine(strBuf);

            ulong endTimestamp = 0;
            ulong startTimestamp = 0;
            ulong deviceTimestamp = 0;
            ulong frameInterval = 0;
            ulong frameIntervalNTP = 0;
            int currentPid = GetCurrentProcessId();
            float memUsage = 0;
            float cpuUsage = 0;
            int number = 100;
            for (int i = 0; i < number; i++)
            {
                //call the below api to trigger one frame detect result, then the result will be sent
                status = VNAPI.VN_AIModuleTriggerOnce(deviceHandle);
                DateTime tTime = DateTime.UtcNow;
                startTimestamp = (ulong)(tTime.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_AIModuleTriggerOnce] fail status:(" + status + ").");
                    continue;
                }
                status = VNAPI.VN_AIModuleGetResult(deviceHandle, 1200, ref pAIResult);
                DateTime currentTime = DateTime.UtcNow;
                endTimestamp = (ulong)(currentTime.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_AIModuleGetResult] fail status:(" + status + ").");
                    continue;
                }
                if (0 == pAIResult.resultIndex % 10)
                {
                    Console.WriteLine("[VN_AIModuleGetResult] success status:(" + status + "). resultIndex: " + pAIResult.resultIndex + " resultTimestamp: " + pAIResult.resultTimestamp);
                }

                deviceTimestamp = pAIResult.resultTimestamp;
                frameInterval = endTimestamp - startTimestamp;
                frameIntervalNTP = endTimestamp - deviceTimestamp;
                memUsage = (float)GetMemoryUsage(currentPid);
                cpuUsage = (float)GetCpuUsage(currentPid);
                // Console.WriteLine($"当前进程PID: {currentPid}, 内存占用: {memUsage:F2} MB, CPU占用: {cpuUsage}%");
                if (frameIntervalNTP > 2000)
                {
                    strBuf = pAIResult.resultIndex.ToString() + "," + cpuUsage + "," + memUsage + "," + frameInterval.ToString() + ",N/A";
                    csvWriter.WriteLine(strBuf);
                }
                else
                {
                    strBuf = pAIResult.resultIndex.ToString() + "," + cpuUsage + "," + memUsage + "," + frameInterval.ToString() + ","+ frameIntervalNTP.ToString();
                    csvWriter.WriteLine(strBuf);
                }
                Thread.Sleep(1200 / frameRate);
            }

            csvWriter.Flush();
            csvWriter.Close();

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
            status = VNAPI.VN_AIModuleSetWorkMode(deviceHandle, ScAIModuleMode.AI_SINGLE_RUN_MODE);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_AIModuleSetWorkMode] success status:(" + status + "). Set AI_SINGLE_RUN_MODE.");
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
