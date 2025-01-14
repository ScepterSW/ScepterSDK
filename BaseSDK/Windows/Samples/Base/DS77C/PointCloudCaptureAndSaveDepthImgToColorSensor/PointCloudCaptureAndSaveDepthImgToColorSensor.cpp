﻿#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>

#define frameSpace 20

using namespace std;

int main()
{
	cout << "---PointCloudCaptureAndSaveDepthImgToColorSensor---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	ScFrameReady frameReady = { 0 };
	ScFrame frame = { 0 };

	status = scInitialize();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scInitialize] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scInitialize] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scGetDeviceCount(&deviceCount, 3000);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceCount] success, ScStatus(" << status << "). The device count is " << deviceCount << endl;
	}
	else
	{
		cout << "[scGetDeviceCount] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}
	if (0 == deviceCount)
	{
		cout << "[scGetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples." << endl;
		return -1;
	}

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceInfoList] success, ScStatus(" << status << ").";
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << " The first device [status]: " << pDeviceListInfo[0].status << " does not support connection." << endl;
			delete[] pDeviceListInfo;
			pDeviceListInfo = NULL;
			return -1;
		}
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
		return -1;
	}

	cout << " The first deviceInfo, <serialNumber>: " << pDeviceListInfo[0].serialNumber
		<< ", <ip>: " << pDeviceListInfo[0].ip << ", <status>: " << pDeviceListInfo[0].status << endl;

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scOpenDeviceBySN] success, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
	}
	else
	{
		cout << "[scOpenDeviceBySN] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
		return -1;
	}

	int resolution_w = 800;
	int resolution_h = 600;
	status = scSetColorResolution(deviceHandle, resolution_w, resolution_h);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetColorResolution] success, ScStatus(" << status << "). Set color sensor resolution 800 * 600." << endl;
	}
	else
	{
		cout << "[scSetColorResolution] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scStartStream(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStartStream] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStartStream] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scSetTransformDepthImgToColorSensorEnabled(deviceHandle, true);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetTransformDepthImgToColorSensorEnabled] success, ScStatus(" << status << "). Enable transform depth frame to color frame." << endl;
	}
	else
	{
		cout << "[scSetTransformDepthImgToColorSensorEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	for (int i = 0; i < frameSpace; i++)
	{
		status = scGetFrameReady(deviceHandle, 1200, &frameReady);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scGetFrameReady] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scGetFrameReady] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		if (1 == frameReady.transformedDepth)
		{
			status = scGetFrame(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &frame);
			if (status == ScStatus::SC_OK)
			{
				cout << "[scGetFrame] success, ScStatus(" << status << "). SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME <frameIndex>: " << frame.frameIndex << endl;

				ofstream pointCloudWriter;
				pointCloudWriter.open("PointCloud.txt");
				ScFrame &srcFrame = frame;
				const int len = srcFrame.width * srcFrame.height;
				ScVector3f* worldV = new ScVector3f[len];
				status = scConvertDepthFrameToPointCloudVector(deviceHandle, &srcFrame, worldV);
				if (status == ScStatus::SC_OK)
				{
					cout << "[scConvertDepthFrameToPointCloudVector] success, ScStatus(" << status << ")." << endl;
				}
				else
				{
					cout << "[scConvertDepthFrameToPointCloudVector] fail, ScStatus(" << status << ")." << endl;
					delete[] worldV;
					worldV = NULL;
					continue;
				}
				for (int i = 0; i < len; i++)
				{
					if (0 < worldV[i].z && worldV[i].z < 0xFFFF)
					{
						pointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << endl;
					}
				}
				pointCloudWriter.close();
				cout << "Save point cloud successful in PointCloud.txt" << endl;
				delete[] worldV;
				worldV = NULL;
				break;
			}
			else
			{
				cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
				return -1;
			}
		}
	}

	status = scStopStream(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStopStream] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStopStream] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scCloseDevice(&deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scCloseDevice] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scCloseDevice] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scShutdown();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scShutdown] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scShutdown] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}
	cout << "---End---" << endl;

	return 0;
}
