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
        public static void DisplayKeyconfiguration(ref ScDeviceHandle deviceHandle, ref ScepterAPI VNAPI)
        {

            ScStatus status = ScStatus.SC_OTHERS;
            IntPtr Version = Marshal.AllocHGlobal(63);
            status = VNAPI.VN_GetSDKVersion(Version, 63);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetEnabled failed status:" + status);
                return;
            }
            string strcVersion = Marshal.PtrToStringAnsi(Version, 63);
            Console.WriteLine("*******VN_GetSDKVersion: " + strcVersion + " ********");

            status = VNAPI.VN_GetFirmwareVersion(deviceHandle, Version, 63);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetFirmwareVersion failed status:" + status);
                return;
            }
            strcVersion = Marshal.PtrToStringAnsi(Version, 63);
            Console.WriteLine("*******VN_GetFirmwareVersion: " + strcVersion + " ********");
            Marshal.FreeHGlobal(Version);
            byte enable = 0;
            status = VNAPI.VN_AIModuleGetEnabled(deviceHandle, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetEnabled: " + enable + "********");
            if (enable != 0)
            {
                Console.WriteLine("******** Config File ERROR, Please check the config file first ********");
                // status = VNAPI.VN_AIModuleSetEnabled(deviceHandle, 0);
                // if (status != ScStatus.SC_OK)
                // {
                //     Console.WriteLine("VN_AIModuleSetEnabled failed status:" + status);
                //     return;
                // }
            }
            status = VNAPI.VN_AIModuleGetInputFrameTypeEnabled(deviceHandle, ScFrameType.SC_COLOR_FRAME, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetInputFrameTypeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetInputFrameTypeEnabled SC_COLOR_FRAME : " + enable + "********");
            status = VNAPI.VN_AIModuleGetInputFrameTypeEnabled(deviceHandle, ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetInputFrameTypeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME : " + enable + "********");

            status = VNAPI.VN_AIModuleGetPreviewFrameTypeEnabled(deviceHandle, ScFrameType.SC_COLOR_FRAME, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetPreviewFrameTypeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetPreviewFrameTypeEnabled SC_COLOR_FRAME : " + enable + "********");
            status = VNAPI.VN_AIModuleGetPreviewFrameTypeEnabled(deviceHandle, ScFrameType.SC_DEPTH_FRAME, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetPreviewFrameTypeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetPreviewFrameTypeEnabled SC_DEPTH_FRAME : " + enable + "********");

            status = VNAPI.VN_AIModuleGetPreviewFrameTypeEnabled(deviceHandle, ScFrameType.SC_IR_FRAME, ref enable);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetPreviewFrameTypeEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("********VN_AIModuleGetPreviewFrameTypeEnabled SC_IR_FRAME : " + enable + "********");
            // get frameRate
            int frameRate = 0;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref frameRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFrameRate failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("scGetFrameRate : " + frameRate);
            }
            int pW = 0;
            int pH = 0;
            status = VNAPI.VN_GetColorResolution(deviceHandle, ref (pW), ref (pH));
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("VN_GetColorResolution pW: " + pW + " pH: " + pH);
            }
        }
        static void Main(string[] args)
        {
            Console.WriteLine("---CameraModelSigleFrameDelayTest---");

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
                    Console.WriteLine("The device state does not support connection.");
                    return;
                }
            }
            else
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return;
            }

            Console.WriteLine("productName:" + pDeviceListInfo[0].productName);
            Console.WriteLine("serialNumber:" + pDeviceListInfo[0].serialNumber);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }
            DisplayKeyconfiguration(ref deviceHandle, ref VNAPI);
            // get frameRate
            int frameRate = 0;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref frameRate);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFrameRate failed status:" + status);
                return;
            }
            //set slave true
            status = VNAPI.VN_SetWorkMode(deviceHandle, ScWorkMode.SC_SOFTWARE_TRIGGER_MODE);
            if (status != ScStatus.SC_OK)

            {
                Console.WriteLine("VN_SetWorkMode failed status:" + status);
                return;
            }
            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            try
            {
                using (StreamWriter sWriter = new StreamWriter("CameraModelSigleFrameDelayTest.csv"))
                {
                    sWriter.Close();
                }
            }
            catch (IOException ex)
            {
                Console.WriteLine("csv file open failed");
                return;
            }

            StreamWriter csvWriter = new StreamWriter("CameraModelSigleFrameDelayTest.csv");
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
            Console.WriteLine("Please input the number of tests:");
            string input = Console.ReadLine(); // 接受用户输入
            int number = 0;
            if (!int.TryParse(input, out number))
            {
                Console.WriteLine("Input error.");
            }

            for (int i = 0; i < number; i++)
            {
                //call the below api to trigger one frame, then the frame will be sent
                // if do not call this function, the frame will not be sent and the below call will return timeout fail
                status = VNAPI.VN_SoftwareTriggerOnce(deviceHandle);
                DateTime tTime = DateTime.UtcNow;
                startTimestamp = (ulong)(tTime.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_SoftwareTriggerOnce failed status:" + status);
                    continue;
                }

                //If no image is ready within 1000ms, the function will return ScRetGetFrameReadyTimeOut
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
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
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetWorkMode failed status:" + status);
                return;
            }

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
            Console.WriteLine("---end---");

            return;
        }
    }
}
