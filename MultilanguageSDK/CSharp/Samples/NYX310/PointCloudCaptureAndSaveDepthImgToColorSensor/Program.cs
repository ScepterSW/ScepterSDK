using System;
using System.IO;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace PointCloudCaptureAndSaveDepthImgToColorSensor
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("---PointCloudCaptureAndSaveDepthImgToColorSensor---");

            //about dev
            ScepterAPI VNAPI = new ScepterAPI();
            UInt32 deviceCount = 0;

            ScDeviceHandle deviceHandle = new IntPtr();
            ScStatus status = ScStatus.SC_OTHERS;

            //about frame

            ScFrameReady FrameReady = new ScFrameReady();

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

            //switch ColorResolution
            int resolution_w = 800;
            int resolution_h = 600;
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetColorResolution] success status:(" + status + "). Set color sensor resolution 800 * 600.");
            }
            else
            {
                Console.WriteLine("[VN_SetColorResolution] fail status:(" + status + ").");
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

            //set Mapper
            status = VNAPI.VN_SetTransformDepthImgToColorSensorEnabled(deviceHandle, 1);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("[VN_SetTransformDepthImgToColorSensorEnabled] success status:(" + status + ").  Enable transform depth frame to color frame");
            }
            else
            {
                Console.WriteLine("[VN_SetTransformDepthImgToColorSensorEnabled] fail status:(" + status + ").");
                return 1;
            }

            //1.ReadNextFrame.
            //2.GetFrame acoording to Ready flag and Frametype.
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
                    ScFrame frame = new ScFrame();
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, ref frame);
                    if (status == ScStatus.SC_OK && frame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("[VN_GetFrame] success status:(" + status + "). SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME <frameIndex>: " + frame.frameIndex);
                        // once save
                        int len = frame.width * frame.height;
                        ScVector3f[] worldV = new ScVector3f[len];
                        frame.frameType = ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME;

                        status = VNAPI.VN_ConvertDepthFrameToPointCloudVector(deviceHandle, ref frame, ref worldV[0]); //Convert Depth frame to World vectors.
                        if (status == ScStatus.SC_OK)
                        {
                            Console.WriteLine("[VN_ConvertDepthFrameToPointCloudVector] success status:(" + status + ").");
                            try
                            {
                                StreamWriter PointCloudWriter = new StreamWriter("PointCloud.txt");
                                for (int index = 0; index < len; index++)
                                {
                                    if (0 < worldV[index].z && worldV[index].z < 0xFFFF)
                                    {
                                        string strBuf = worldV[index].x + "\t" + worldV[index].y + "\t" + worldV[index].z;
                                        PointCloudWriter.WriteLine(strBuf);
                                    }
                                }
                                PointCloudWriter.Flush();
                                PointCloudWriter.Close();
                            }
                            catch (Exception e)
                            {
                                Console.WriteLine("Exception: " + e.Message);
                            }
                            finally
                            {
                                Console.WriteLine("Save point cloud successful in PointCloud.txt");
                            }
                            break;
                        }
                        else
                        {
                            Console.WriteLine("[VN_ConvertDepthFrameToPointCloudVector] fail status:(" + status + ").");
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
