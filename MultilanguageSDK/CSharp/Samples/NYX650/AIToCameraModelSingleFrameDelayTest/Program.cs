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
            Console.WriteLine("---CameraModelSingleFrameDelayTest---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about frame

            ScFrameReady FrameReady = new ScFrameReady();
            ScFrame depthFrame = new ScFrame();

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
            //set slave true
            status = VNAPI.VN_SetWorkMode(deviceHandle, ScWorkMode.SC_SOFTWARE_TRIGGER_MODE);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetWorkMode] success status:(" + status + "). Set SC_SOFTWARE_TRIGGER_MODE.");
            }
            else
            {
                Console.WriteLine("[VN_SetWorkMode] fail status:(" + status + ").");
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
                using (StreamWriter sWriter = new StreamWriter("CameraModelSingleFrameDelayTest.csv"))
                {
                    sWriter.Close();
                }
            }
            catch (IOException ex)
            {
                Console.WriteLine("csv file open failed" + ex.Message);
                return 1;
            }

            StreamWriter csvWriter = new StreamWriter("CameraModelSingleFrameDelayTest.csv");
            string strBuf = "frameIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure";
            csvWriter.WriteLine(strBuf);

            //1.software trigger.
            //2.ReadNextFrame.
            //3.GetFrame acoording to Ready flag and Frametype.
            //4.sleep 1000/frameRate (ms)
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
                //call the below api to trigger one frame, then the frame will be sent
                // if do not call this function, the frame will not be sent and the below call will return timeout fail
                status = VNAPI.VN_SoftwareTriggerOnce(deviceHandle);
                DateTime tTime = DateTime.UtcNow;
                startTimestamp = (ulong)(tTime.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_SoftwareTriggerOnce] fail status:(" + status + ").");
                    continue;
                }

                //If no image is ready within 1000ms, the function will return ScRetGetFrameReadyTimeOut
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_GetFrameReady] fail status:(" + status + ").");
                    continue;
                }

                //depthFrame for example.
                if (1 == FrameReady.depth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref depthFrame);
                    if (depthFrame.pFrameData != IntPtr.Zero)
                    {
                        if (0 == depthFrame.frameIndex % 10)
                        {
                            Console.WriteLine("VN_GetFrame,status:" + status + "  "
                                + "frameType:" + depthFrame.frameType + "  "
                                + "frameIndex:" + depthFrame.frameIndex);
                            Console.WriteLine("[VN_GetFrame] success status:(" + status + "). frameType: " + depthFrame.frameType + " resultTimestamp: " + depthFrame.frameIndex);
                        }
                        DateTime currentTime = DateTime.UtcNow;
                        endTimestamp = (ulong)(currentTime.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
                        deviceTimestamp = depthFrame.deviceTimestamp;
                        frameInterval = endTimestamp - startTimestamp;
                        frameIntervalNTP = endTimestamp - deviceTimestamp;
                        memUsage = (float)GetMemoryUsage(currentPid);
                        cpuUsage = (float)GetCpuUsage(currentPid);
                        // Console.WriteLine($"当前进程PID: {currentPid}, 内存占用: {memUsage:F2} MB, CPU占用: {cpuUsage}%");
                        if (frameIntervalNTP > 2000)
                        {
                            strBuf = depthFrame.frameIndex.ToString() + "," + cpuUsage + "," + memUsage + "," + frameInterval.ToString() + ",N/A";
                            csvWriter.WriteLine(strBuf);
                        }
                        else
                        {
                            strBuf = depthFrame.frameIndex.ToString() + "," + cpuUsage + "," + memUsage + "," + frameInterval.ToString() + ","+ frameIntervalNTP.ToString();
                            csvWriter.WriteLine(strBuf);
                        }
                    }
                }
                //The minimum time interval to trigger a signal is 1000/FPS milliseconds
                Thread.Sleep(1000 / frameRate);
            }

            csvWriter.Flush();
            csvWriter.Close();
            //set slave false
            status = VNAPI.VN_SetWorkMode(deviceHandle, ScWorkMode.SC_ACTIVE_MODE);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetWorkMode] success status:(" + status + "). Set SC_ACTIVE_MODE.");
            }
            else
            {
                Console.WriteLine("[VN_SetWorkMode] fail status:(" + status + ").");
                return 1;
            }

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
    }
}
