﻿using System;
using System.IO;
using System.Text;
using System.Threading;
using Scepter_enums;
using Scepter_types;
using Scepter_api;

namespace PointCloudVectorAndSaveDepthImgToColorSensor
{
    using ScDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---PointCloudVectorAndSaveDepthImgToColorSensor---");

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
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo); ;
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

            //switch ColorResolution
            int resolution_w = 800;
            int resolution_h = 600;
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 800_600");
            }

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != ScStatus.SC_OK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }
            //set Mapper
            status = VNAPI.VN_SetTransformDepthImgToColorSensorEnabled(deviceHandle, 1);
            if (status == ScStatus.SC_OK)
            {
                Console.WriteLine("VN_SetTransformDepthImgToColorSensorEnabled Enabled");
            }

            ScSensorIntrinsicParameters cameraParam = new ScSensorIntrinsicParameters();
            VNAPI.VN_GetSensorIntrinsicParameters(deviceHandle, ScSensorType.SC_COLOR_SENSOR, ref cameraParam);

            //1.ReadNextFrame.
            //2.GetFrame acoording to Ready flag and Frametype.
            //3.save points.
            for (int indext = 0; indext < 20; indext++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != ScStatus.SC_OK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                if (1 == FrameReady.transformedDepth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, ref frame);
                    if (status == ScStatus.SC_OK && frame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("VN_GetFrame,status:" + status + "  "
                            + "frameType:" + frame.frameType + "  "
                            + "frameIndex:" + frame.frameIndex);

                        // once save
                        try
                        {

                            StreamWriter PointCloudWriter = new StreamWriter("PointCloud.txt");
                            ScFrame srcFrame = frame;

                            Console.WriteLine(srcFrame.width + "," + srcFrame.height);

                            char[] pDepthFrameData = new char[frame.dataLen];
                            int iLen = (int)frame.dataLen;
                            System.Runtime.InteropServices.Marshal.Copy(frame.pFrameData, pDepthFrameData, 0, iLen / 2);
                            for (int i = 0, offset = i * srcFrame.width; i < srcFrame.height; i++)
                            {
                                for (int j = 0; j < srcFrame.width; j++)
                                {
                                    ScDepthVector3 depthPoint = new ScDepthVector3();
                                    depthPoint.depthX = j;
                                    depthPoint.depthY = i;
                                    depthPoint.depthZ = pDepthFrameData[offset + j];
                                    ScVector3f worldV = new ScVector3f();
                                    VNAPI.VN_ConvertDepthToPointCloud(deviceHandle, ref depthPoint, ref worldV, 1, ref cameraParam);
                                    if (0 < worldV.z && worldV.z < 0xFFFF)
                                    {
                                        string strBuf = worldV.x + "\t" + worldV.y + "\t" + worldV.z;
                                        PointCloudWriter.WriteLine(strBuf);
                                    }
                                }
                                offset += srcFrame.width;
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
                }

            }


            //StoSc capturing the image stream
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
