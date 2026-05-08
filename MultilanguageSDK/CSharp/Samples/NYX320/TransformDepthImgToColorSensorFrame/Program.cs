using System;
using System.IO;
using System.Text;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace TransformDepthImgToColorSensorFrame
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---TransformDepthImgToColorSensorFrame---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;

            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about frame

            ScFrameReady FrameReady = new ScFrameReady();
            ScFrame frame = new ScFrame();

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

            //Wait for the device to upload image data
            Thread.Sleep(1000);

            //set Mapper
            status = VNAPI.VN_SetTransformDepthImgToColorSensorEnabled(deviceHandle, 1);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetTransformDepthImgToColorSensorEnabled] success status:(" + status + ").");
            }
            else
            {
                Console.WriteLine("[VN_SetTransformDepthImgToColorSensorEnabled] fail status:(" + status + ").");
                return 1;
            }

            //1.ReadNextFrame.
            //2.GetFrame acoording to Ready flag and Frametype.
            bool saveonce = false;
            for (int i = 0; i < 20; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status == ScStatus.SC_OK)
                {
                    Console.WriteLine("[VN_GetFrameReady] success status:(" + status + ").");
                }
                else
                {
                    Console.WriteLine("[VN_GetFrameReady] fail status:(" + status + ").");
                    continue;
                }

                if (1 == FrameReady.transformedDepth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, ref frame);
                    if (status == ScStatus.SC_OK && frame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("[VN_GetFrame] success status:(" + status + "). SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME <frameIndex>: " + frame.frameIndex);

                        if (!saveonce)
                        { //name of the frames to be saved.
                            string fname = "TransformDepthImgToColorSensorFrame_" + frame.frameIndex + ".bin";

                            //save
                            try
                            {
                                BinaryWriter sw = new BinaryWriter(new FileStream(fname, FileMode.Create));
                                byte[] buf = new byte[frame.dataLen];
                                int iLen = (int)frame.dataLen;
                                System.Runtime.InteropServices.Marshal.Copy(frame.pFrameData, buf, 0, iLen);
                                sw.Write(buf, 0, iLen);
                                sw.Flush();
                                sw.Close();
                                Console.WriteLine("Save TransformDepthImgToColorSensorFrame successful in " + fname);
                                saveonce = true;
                            }
                            catch (Exception e)
                            {
                                Console.WriteLine("Exception: " + e.Message);
                            }
                        }
                    }
                    else
                    {
                        Console.WriteLine("[VN_GetFrame] fail status:(" + status + ").");
                    }
                }

            }
            //Stop capturing the image stream
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
