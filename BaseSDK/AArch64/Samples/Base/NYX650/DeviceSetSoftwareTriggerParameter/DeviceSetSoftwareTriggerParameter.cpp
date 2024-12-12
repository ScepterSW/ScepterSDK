﻿#include <thread>
#include <iostream>
#include "Scepter_api.h"

#define frameSpace 10
using namespace std;

int main()
{
	cout << "---DeviceSetSoftwareTriggerParameter---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	ScFrameReady frameReady = { 0 };
	ScFrame depthFrame = { 0 };
	bool bSlaveEnabled = true;

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

	status = scSetWorkMode(deviceHandle, SC_SOFTWARE_TRIGGER_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetWorkMode] success, ScStatus(" << status << "). Set SC_SOFTWARE_TRIGGER_MODE." << endl;
	}
	else
	{
		cout << "[scSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int frameRate = 5;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameRate] success, ScStatus(" << status << "). The device frame rate is " << frameRate << endl;
	}
	else
	{
		cout << "[scGetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	uint8_t value = 5;
	status = scSetSoftwareTriggerParameter(deviceHandle, value);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetSoftwareTriggerParameter] success, ScStatus(" << status << "). Set the device software trigger parameter to " << (int)value << endl;
	}
	else
	{
		cout << "[scSetSoftwareTriggerParameter] fail, ScStatus(" << status << ")." << endl;
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

	cout << "Software trigger test begins." << endl;
	for (int i = 0; i < frameSpace; i++)
	{
		status = scSoftwareTriggerOnce(deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scSoftwareTriggerOnce] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scSoftwareTriggerOnce] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		//The trigger parameter is set to 5, and the device will collect 5 
		//images and report the best one, so the waiting time needs to be set longer here.
		status = scGetFrameReady(deviceHandle, 15000, &frameReady);
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
			}
			else
			{
				cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
			}
		}

		//The time interval between two triggers should be greater than 
		//the time interval between the two frames generated.
		this_thread::sleep_for(chrono::milliseconds(1000 / frameRate));
	}

	status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetWorkMode] success, ScStatus(" << status << "). Set SC_ACTIVE_MODE." << endl;
	}
	else
	{
		cout << "[scSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
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