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

        static void Main(string[] args)
        {
            Console.WriteLine("---AIGetFrameAndResultSync---");

            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;
            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //Initialize the ScepterSDK.
            status = VNAPI.VN_Initialize();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Initialize failed status:" + status);
                Console.ReadKey(true);
                return;
            }

            //Get the count of devices.
            status = VNAPI.VN_GetDeviceCount(ref deviceCount, 3000);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_GetDeviceCount failed! make sure pointer valid or called VN_Initialize()");
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

            //Get the infomation of devices.
            ScDeviceInfo[] pDeviceListInfo = new ScDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status == ScStatus.SC_OK)
            {
                if (ScConnectStatus.SC_CONNECTABLE != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect status: " + pDeviceListInfo[0].status);
                    Console.WriteLine("The device state does not support connection.");
                    return;
                }
            }
            else
            {
                Console.WriteLine("VN_GetDeviceInfoList failed status:" + status);
                return;
            }

            Console.WriteLine("serialNumber:" + pDeviceListInfo[0].serialNumber);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            //Open the first device by serial number.
            status = VNAPI.VN_OpenDeviceBySN(pDeviceListInfo[0].serialNumber, ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_OpenDeviceBySN failed status:" + status);
                return;
            }

            //Set bSetParams to false. if the params has been initialized by "Parameter initialization file" in ScepterGUITool or by SDK API in "xxxx" sample.
            bool bSetParams = true;
            if (bSetParams)
            {
                status = SetParams(deviceHandle, ref VNAPI);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("SetParams failed status:" + status);
                    return;
                }
            }

            //Start the data stream.
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
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
                            Console.WriteLine("Sync result fail, resultIndex: " + tAIResult.resultIndex + ", timestamp: " + tAIResult.resultTimestamp + ", frameIndex:" + tFrame.frameIndex + ", timestamp: " + tFrame.deviceTimestamp);
                            resultQueue.Dequeue();
                        }
                        else if (tFrame.deviceTimestamp < tAIResult.resultTimestamp)
                        {
                            Console.WriteLine("Sync frame fail, frameIndex:" + tFrame.frameIndex + ", timestamp: " + tFrame.deviceTimestamp + ", resultIndex: " + tAIResult.resultIndex + ", timestamp : " + tAIResult.resultTimestamp);
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
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StopStream failed status:" + status);
                return;
            }

            //Close the opened device.
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_CloseDevice failed status:" + status);
                return;
            }

            //Shutdown the initialized ScepterSDK.
            status = VNAPI.VN_Shutdown();
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }
            Console.WriteLine("---end---");

            return;
        }

        public static ScStatus SetParams(ScDeviceHandle deviceHandle, ref ScepterAPI VNAPI)
        {
            ScStatus status = ScStatus.SC_OTHERS;
            //Set frame rate.
            status = VNAPI.VN_SetFrameRate(deviceHandle, 15);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetFrameRate failed status:" + status);
                return status;
            }
            //Set color resolution.
            status = VNAPI.VN_SetColorResolution(deviceHandle, 640, 480);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return status;
            }

            //Get the parameter with the parameter ID 4.
            UInt32 paramID = 4;
            IntPtr pBuffer = new IntPtr();
            ushort nBufferSize = 0;
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetParam paramID: 4 failed status:" + status);
                return status;
            }
            string strOldASCII = Marshal.PtrToStringAnsi(pBuffer, nBufferSize);
            Console.WriteLine("VN_AIModuleGetParam paramID: 4 status:" + status + "  text(ASCII): " + strOldASCII);

            //Set the parameter with the parameter ID 4.
            string strASCII = "Hello world.";
            byte[] myByteArray = new byte[strASCII.Length];
            myByteArray = System.Text.Encoding.Default.GetBytes(strASCII);
            IntPtr cASCII = System.Runtime.InteropServices.Marshal.UnsafeAddrOfPinnedArrayElement(myByteArray, 0);
            Int32 ASCIISize = strASCII.Length;
            ushort uASCIISize = (ushort)ASCIISize;
            status = VNAPI.VN_AIModuleSetParam(deviceHandle, paramID, cASCII, uASCIISize);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetParam paramID: 4 failed status:" + status);
                return status;
            }
            Console.WriteLine("VN_AIModuleSetParam paramID: 4 status:" + status + "  text(ASCII): " + strASCII);

            //Get the parameter with the parameter ID 4.
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleGetParam paramID: 4 failed status:" + status);
                return status;
            }
            string strNewASCII = Marshal.PtrToStringAnsi(pBuffer, nBufferSize);
            Console.WriteLine("VN_AIModuleGetParam paramID: 4 status:" + status + "  text(ASCII): " + strNewASCII);

            //Get the parameter with the parameter ID 5.
            paramID = 5;
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status != ScStatus.SC_OK || nBufferSize < 1)
            {
                Console.WriteLine("VN_AIModuleGetParam paramID: 5 failed status:" + status);
                return status;
            }
            byte[] myOldAnsiArray = new byte[nBufferSize];
            Marshal.Copy(pBuffer, myOldAnsiArray, 0, nBufferSize);
            Console.Write("VN_AIModuleGetParam paramID: 5 status:" + status + "  text(HEX): ");
            for (int i = 0; i < nBufferSize; i++)
            {
                string hexValue = myOldAnsiArray[i].ToString("X2");
                Console.Write("0x" + hexValue + " ");
            }
            Console.WriteLine();

            //Set the parameter with the parameter ID 5.
            byte[] myHEXByteArray = { 0x01, 0x02, 0x03 };
            IntPtr cHEXII = System.Runtime.InteropServices.Marshal.UnsafeAddrOfPinnedArrayElement(myHEXByteArray, 0);
            status = VNAPI.VN_AIModuleSetParam(deviceHandle, paramID, cHEXII, 3);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetParam paramID: 5 failed status:" + status);
                return status;
            }
            Console.Write("VN_AIModuleGetParam paramID: 5 status:" + status + "  text(HEX): ");
            for (int i = 0; i < 3; i++)
            {
                string hexValue = myHEXByteArray[i].ToString("X2");
                Console.Write("0x" + hexValue + " ");
            }
            Console.WriteLine();

            //Get the parameter with the parameter ID 5.
            paramID = 5;
            status = VNAPI.VN_AIModuleGetParam(deviceHandle, paramID, ref pBuffer, ref nBufferSize);
            if (status != ScStatus.SC_OK || nBufferSize < 1)
            {
                Console.WriteLine("VN_AIModuleGetParam paramID: 5 failed status:" + status);
                return status;
            }

            byte[] myNewAnsiArray = new byte[nBufferSize];
            Marshal.Copy(pBuffer, myNewAnsiArray, 0, nBufferSize);
            Console.Write("VN_AIModuleGetParam paramID: 5 status:" + status + "  text(HEX): ");
            for (int i = 0; i < nBufferSize; i++)
            {
                string hexValue = myNewAnsiArray[i].ToString("X2");
                Console.Write("0x" + hexValue + " ");
            }
            Console.WriteLine();

            //Set input frame type of SC_COLOR_FRAME enabled.
            status = VNAPI.VN_AIModuleSetInputFrameTypeEnabled(deviceHandle, ScFrameType.SC_COLOR_FRAME, 1);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetInputFrameTypeEnabled color failed status:" + status);
                return status;
            }

            //Set input frame type of SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME enabled.
            status = VNAPI.VN_AIModuleSetInputFrameTypeEnabled(deviceHandle, ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, 1);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetInputFrameTypeEnabled transforedDepth  failed status:" + status);
                return status;
            }

            //Set preview frame type of SC_DEPTH_FRAME disabled.
            status = VNAPI.VN_AIModuleSetPreviewFrameTypeEnabled(deviceHandle, ScFrameType.SC_DEPTH_FRAME, 0);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetPreviewFrameTypeEnabled depth failed status:" + status);
                return status;
            }

            //Set preview frame type of SC_IR_FRAME disabled.
            status = VNAPI.VN_AIModuleSetPreviewFrameTypeEnabled(deviceHandle, ScFrameType.SC_IR_FRAME, 0);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetPreviewFrameTypeEnabled IR failed status:" + status);
                return status;
            }

            //Set AI module continuous running.
            status = VNAPI.VN_AIModuleSetWorkMode(deviceHandle, ScAIModuleMode.AI_CONTINUOUS_RUN_MODE);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetWorkMode failed status:" + status);
                return status;
            }

            //Enable AI module.
            status = VNAPI.VN_AIModuleSetEnabled(deviceHandle, 1);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_AIModuleSetEnabled failed status:" + status);
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
                        ScFrame colorFrame = new ScFrame();
                        ScFrameReady frameReady = new ScFrameReady();
                        ScStatus status = VNAPI.VN_GetFrameReady(device_, 1000, ref frameReady);
                        if (status != ScStatus.SC_OK)
                        {
                            Console.WriteLine("VN_GetFrameReady failed status:" + status);
                            continue;
                        }

                        // Get color frame.
                        if (1 == frameReady.color)
                        {
                            status = VNAPI.VN_GetFrame(device_, ScFrameType.SC_COLOR_FRAME, ref colorFrame);
                            if (status == ScStatus.SC_OK && colorFrame.pFrameData != IntPtr.Zero)
                            {
                                frameMutex_.WaitOne();
                                if (frameQueue_.Count >= 6)
                                {
                                    frameQueue_.Dequeue();
                                    frameQueue_.Enqueue(colorFrame);
                                }
                                else
                                {
                                    frameQueue_.Enqueue(colorFrame);
                                }
                                frameMutex_.ReleaseMutex();
                            }
                            else
                            {
                                Console.WriteLine("scGetFrame SC_COLOR_FRAME status:" + status);
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
                            Console.WriteLine("VN_AIModuleGetResult failed status:" + status);
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
