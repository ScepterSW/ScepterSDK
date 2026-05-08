using System;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace AIGetFrameAndResultSync
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        private static Mutex frameMutex = new Mutex();
        private static Mutex resultMutex = new Mutex();
        private static Queue<ScFrame> frameQueue = new Queue<ScFrame>();
        private static Queue<ScAIResult> resultQueue = new Queue<ScAIResult>();

        static int Main(string[] args)
        {
            Console.WriteLine("---AIGetFrameAndResultSync---");

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
            
            //Create two threads to get the frame and the result of AI mode respectively.
            DealThread[] cDealThread = new DealThread[2];
            //Get frame.
            cDealThread[0] = new DealThread(deviceHandle, 1, true, ref VNAPI, frameQueue, resultQueue, frameMutex, resultMutex);
            //Get result of AI mode.
            cDealThread[1] = new DealThread(deviceHandle, 2, true, ref VNAPI, frameQueue, resultQueue, frameMutex, resultMutex);
            Thread[] t = new Thread[2];
            for (UInt32 i = 0; i < 2; i++)
            {
                t[i] = new Thread(cDealThread[i].Deal);
                t[i].Start();
            }

            //sync.
            ScFrame tFrame = new ScFrame();
            ScAIResult tAIResult = new ScAIResult();
            int syncCount = 100;
            while (syncCount > 0)
            {
                {
                    resultMutex.WaitOne();
                    frameMutex.WaitOne();
                    while (0 != resultQueue.Count && 0 != frameQueue.Count)
                    {
                        tAIResult = resultQueue.Peek();
                        tFrame = frameQueue.Peek();

                        if (tFrame.deviceTimestamp == tAIResult.resultTimestamp)
                        {
                            Console.WriteLine("Sync frame index:" + tFrame.frameIndex + ", result index: " + tAIResult.resultIndex + ", timestamp: " + tAIResult.resultTimestamp);
                            frameQueue.Dequeue();
                            resultQueue.Dequeue();
                            syncCount--;
                            break;
                        }
                        else if (tFrame.deviceTimestamp > tAIResult.resultTimestamp)
                        {
                            Console.WriteLine("Sync result not match bigger, resultIndex: " + tAIResult.resultIndex + ", timestamp: " + tAIResult.resultTimestamp + ", frameIndex:" + tFrame.frameIndex + ", timestamp: " + tFrame.deviceTimestamp);
                            resultQueue.Dequeue();
                        }
                        else if (tFrame.deviceTimestamp < tAIResult.resultTimestamp)
                        {
                            Console.WriteLine("Sync frame not match less, frameIndex:" + tFrame.frameIndex + ", timestamp: " + tFrame.deviceTimestamp + ", resultIndex: " + tAIResult.resultIndex + ", timestamp : " + tAIResult.resultTimestamp);
                            frameQueue.Dequeue();
                        }
                    }
                    resultMutex.ReleaseMutex();
                    frameMutex.ReleaseMutex();
                }
                Thread.Sleep(10);
            }
            
            //Wait for threads done.
            for (UInt32 i = 0; i < 2; i++)
            {
                cDealThread[i].SetRunning(false);
                cDealThread[i].WaitForTestDone();
                t[i].Abort();
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
    
    class DealThread
    {
        public DealThread(ScDeviceHandle device, Int32 type, bool running, ref ScepterAPI pVNAPI, Queue<ScFrame> frameQueue, Queue<ScAIResult> resultQueue, Mutex frameMutex, Mutex resultMutex)
        {
            device_ = device;
            type_ = type;
            running_ = running;
            VNAPI = pVNAPI;
            frameQueue_ = frameQueue;
            resultQueue_ = resultQueue;
            frameMutex_ = frameMutex;
            resultMutex_ = resultMutex;
        }
        public void Deal()
        {
            lock (this)
            {
                // type: 1. get frame;2. get result of AI mode.
                if (1 == type_)
                {
                    while (running_)
                    {
                        ScFrame depthFrame = new ScFrame();
                        ScFrameReady frameReady = new ScFrameReady();
                        ScStatus status = VNAPI.VN_GetFrameReady(device_, 1000, ref frameReady);
                        if (status != ScStatus.SC_OK)
                        {
                            Console.WriteLine("[VN_GetFrameReady] fail status:(" + status + ").");
                            continue;
                        }

                        // Get depth frame.
                        if (1 == frameReady.depth)
                        {
                            status = VNAPI.VN_GetFrame(device_, ScFrameType.SC_DEPTH_FRAME, ref depthFrame);
                            if (status == ScStatus.SC_OK && depthFrame.pFrameData != IntPtr.Zero)
                            {
                                frameMutex_.WaitOne();
                                if (frameQueue_.Count >= 6)
                                {
                                    frameQueue_.Dequeue();
                                    frameQueue_.Enqueue(depthFrame);
                                }
                                else
                                {
                                    frameQueue_.Enqueue(depthFrame);
                                }
                                frameMutex_.ReleaseMutex();
                            }
                            else
                            {
                                Console.WriteLine("[VN_GetFrame] fail status:(" + status + ").");
                            }
                        }
                    };
                }
                else
                {
                    while (running_)
                    {
                        ScAIResult aiResult = new ScAIResult();
                        ScStatus status = VNAPI.VN_AIModuleGetResult(device_, 1000, ref aiResult);
                        if (status != ScStatus.SC_OK)
                        {
                            Console.WriteLine("[VN_AIModuleGetResult] fail status:(" + status + ").");
                            continue;
                        }
                        {
                            resultMutex_.WaitOne();
                            if (resultQueue_.Count >= 6)
                            {
                                resultQueue_.Dequeue();
                                resultQueue_.Enqueue(aiResult);
                            }
                            else
                            {
                                resultQueue_.Enqueue(aiResult);
                            }
                            resultMutex_.ReleaseMutex();
                        }
                    };
                }
                isTestDone_ = true;
            }
            return;
        }

        public bool WaitForTestDone()
        {
            while (!isTestDone_)
            {
                Thread.Sleep(1000);
            }
            return true;
        }

        public void SetRunning(bool running)
        {
            running_ = running;
        }
        
        ScDeviceHandle device_;
        bool isTestDone_ = false;
        static ScepterAPI VNAPI;
        Int32 type_;
        bool running_;
        Mutex frameMutex_;
        Mutex resultMutex_;
        Queue<ScFrame> frameQueue_;
        Queue<ScAIResult> resultQueue_;
    }
}
