﻿#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>

#define frameSpace 20
using namespace std;

int main()
{
	cout << "---PointCloudVectorAndSave---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	ScFrameReady frameReady = { 0 };
	ScFrame depthFrame = { 0 };
	ofstream pointCloudWriter;

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

	ScSensorIntrinsicParameters cameraParam = { 0 };
	status = scGetSensorIntrinsicParameters(deviceHandle, SC_TOF_SENSOR, &cameraParam);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetSensorIntrinsicParameters] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scGetSensorIntrinsicParameters] fail, ScStatus(" << status << ")." << endl;
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

	//Wait for the device to upload image data.
	this_thread::sleep_for(chrono::milliseconds(1000));

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

		if (1 == frameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (status == ScStatus::SC_OK)
			{
				cout << "[scGetFrame] success, ScStatus(" << status << "). SC_DEPTH_FRAME <frameIndex>: " << depthFrame.frameIndex << endl;

				pointCloudWriter.open("PointCloud.txt");
				ScFrame &srcFrame = depthFrame;
				const int WINDOW_SIZE = 100;
				const uint16_t* pDepthFrameData = (uint16_t*)srcFrame.pFrameData;
				for (int i = (srcFrame.height - WINDOW_SIZE) / 2, offset = i * srcFrame.width; i < (srcFrame.height + WINDOW_SIZE) / 2; i++)
				{
					for (int j = (srcFrame.width - WINDOW_SIZE) / 2; j < (srcFrame.width + WINDOW_SIZE) / 2; j++)
					{
						ScDepthVector3 depthPoint = { j, i, pDepthFrameData[offset + j] };
						ScVector3f worldV = {};
						status = scConvertDepthToPointCloud(deviceHandle, &depthPoint, &worldV, 1, &cameraParam);
						if (status == ScStatus::SC_OK)
						{
						}
						else
						{
							cout << "[scConvertDepthToPointCloud] fail, ScStatus(" << status << ")." << endl;
						}
						if (0 < worldV.z && worldV.z < 0xFFFF)
						{
							pointCloudWriter << worldV.x << "\t" << worldV.y << "\t" << worldV.z << endl;
						}
					}
					offset += srcFrame.width;
				}
				cout << "Save point cloud successful in PointCloud.txt" << endl;
				pointCloudWriter.close();
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
