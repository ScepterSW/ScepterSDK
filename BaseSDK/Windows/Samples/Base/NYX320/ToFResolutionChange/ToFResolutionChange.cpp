#include <thread>
#include <iostream>
#include "Scepter_api.h"

#define frameSpace 10
using namespace std;

int main()
{
	cout << "---ToFResolutionChange---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	ScFrameReady frameReady = { 0 };
	ScFrame DepthFrame = { 0 };

	status = scInitialize();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scInitialize] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scInitialize] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	status = scGetDeviceCount(&deviceCount, 3000);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceCount] success, ScStatus(" << status << "). The device count is " << deviceCount << endl;
	}
	else
	{
		cout << "[scGetDeviceCount] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	if (0 == deviceCount)
	{
		cout << "[scGetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples." << endl;
		return 1;
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
			return 1;
		}
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
		return 1;
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
		return 1;
	}

	cout << endl << "---Test ToFResolution 640 * 480 ---" << endl;
	int resolution_w = 640;
	int resolution_h = 480;
	status = scSetToFResolution(deviceHandle, resolution_w, resolution_h);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetToFResolution] success, ScStatus(" << status << "). Set tof sensor resolution 640 * 480." << endl;
	}
	else
	{
		cout << "[scSetToFResolution] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	status = scStartStream(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStartStream] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStartStream] fail, ScStatus(" << status << ")." << endl;
		return 1;
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

		if (1 == frameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &DepthFrame);
			if (status == ScStatus::SC_OK)
			{
				cout << "[scGetFrame] success, ScStatus(" << status << "). SC_DEPTH_FRAME <frameIndex>: " << DepthFrame.frameIndex
					<< ", resolution: " << DepthFrame.width << " * " << DepthFrame.height << endl;
			}
			else
			{
				cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
			}
		}
	}

	cout << endl << "---Test ToFResolution 320 * 240 ---" << endl;
	resolution_w = 320;
	resolution_h = 240;
	status = scSetToFResolution(deviceHandle, resolution_w, resolution_h);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetToFResolution] success, ScStatus(" << status << "). Set tof sensor resolution 320 * 240." << endl;
	}
	else
	{
		cout << "[scSetToFResolution] fail, ScStatus(" << status << ")." << endl;
		return 1;
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

		if (1 == frameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &DepthFrame);
			if (status == ScStatus::SC_OK)
			{
				cout << "[scGetFrame] success, ScStatus(" << status << "). SC_DEPTH_FRAME <frameIndex>: " << DepthFrame.frameIndex
					<< ", resolution: " << DepthFrame.width << " * " << DepthFrame.height << endl;
			}
			else
			{
				cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
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
		return 1;
	}

	status = scCloseDevice(&deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scCloseDevice] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scCloseDevice] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	status = scShutdown();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scShutdown] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scShutdown] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	cout << "---End---" << endl;

	return 0;
}
