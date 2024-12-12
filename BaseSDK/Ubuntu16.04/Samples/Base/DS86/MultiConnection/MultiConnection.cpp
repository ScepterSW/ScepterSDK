#include <thread>
#include <iostream>
#include "Scepter_api.h"

#define frameSpace 10
using namespace std;

int main()
{
	cout << "---MultiConnection---" << endl;

	uint32_t deviceCount = 0;
	ScStatus status = SC_OTHERS;
	ScDeviceHandle *deviceHandle = nullptr;
	ScDeviceInfo* pDeviceListInfo = nullptr;
	ScFrameReady frameReady = { 0 };
	ScFrame depthFrame = { 0 };

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

	do
	{
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
	} while (deviceCount < 2);

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceInfoList] success, ScStatus(" << status << ").";
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
		return -1;
	}

	cout << endl;
	for (int i = 0; i < deviceCount; i++)
	{
		cout << " The device index: " << i << ", <serialNumber>: " << pDeviceListInfo[i].serialNumber
			<< ", <ip>: " << pDeviceListInfo[i].ip << ", <status>: " << pDeviceListInfo[i].status << endl;
	}

	deviceHandle = new ScDeviceHandle[deviceCount];
	for (int i = 0; i < deviceCount; i++)
	{
		status = scOpenDeviceBySN(pDeviceListInfo[i].serialNumber, &deviceHandle[i]);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scOpenDeviceBySN] success, ScStatus(" << status << "). The device index: " << i << ", <serialNumber>: " << pDeviceListInfo[i].serialNumber << endl;
		}
		else
		{
			cout << "[scOpenDeviceBySN] fail, ScStatus(" << status << "). The device index: " << i << ", <serialNumber>: " << pDeviceListInfo[i].serialNumber << endl;
			delete[] pDeviceListInfo;
			pDeviceListInfo = NULL;
			delete[] deviceHandle;
			deviceHandle = NULL;
			return -1;
		}
	}
	delete[] pDeviceListInfo;
	pDeviceListInfo = NULL;

	for (int i = 0; i < deviceCount; i++)
	{
		status = scStartStream(deviceHandle[i]);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStartStream] success, ScStatus(" << status << "). The device index: " << i << endl;
		}
		else
		{
			cout << "[scStartStream] fail, ScStatus(" << status << "). The device index: " << i << endl;
			delete[] deviceHandle;
			deviceHandle = NULL;
			return -1;
		}
	}

	//Wait for the device to upload image data.
	this_thread::sleep_for(chrono::milliseconds(1000));

	for (int j = 0; j < frameSpace; j++)
	{
		for (int i = 0; i < deviceCount; i++)
		{
			if (deviceHandle[i])
			{
				status = scGetFrameReady(deviceHandle[i], 1200, &frameReady);
				if (status == ScStatus::SC_OK)
				{
				}
				else
				{
					cout << "[scGetFrameReady] fail, ScStatus(" << status << "). The device index: " << i << endl;
					continue;
				}

				if (1 == frameReady.depth)
				{
					status = scGetFrame(deviceHandle[i], SC_DEPTH_FRAME, &depthFrame);
					if (status == ScStatus::SC_OK)
					{
						cout << "[scGetFrame] success, ScStatus(" << status << "). The device index: " << i << ", SC_DEPTH_FRAME <frameIndex>: " << depthFrame.frameIndex << endl;
					}
					else
					{
						cout << "[scGetFrame] fail, ScStatus(" << status << "). The device index: " << i << endl;
					}
				}
			}
		}
	}

	for (int i = 0; i < deviceCount; i++)
	{
		status = scStopStream(deviceHandle[i]);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStopStream] success, ScStatus(" << status << "). The device index: " << i << endl;
		}
		else
		{
			cout << "[scStopStream] fail, ScStatus(" << status << "). The device index: " << i << endl;
			delete[] deviceHandle;
			deviceHandle = NULL;
			return -1;
		}

		status = scCloseDevice(&deviceHandle[i]);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scCloseDevice] success, ScStatus(" << status << "). The device index: " << i << endl;
		}
		else
		{
			cout << "[scCloseDevice] fail, ScStatus(" << status << "). The device index: " << i << endl;
			delete[] deviceHandle;
			deviceHandle = NULL;
			return -1;
		}
	}
	delete[] deviceHandle;
	deviceHandle = NULL;

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
