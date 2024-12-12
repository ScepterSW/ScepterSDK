#include <iostream>
#include <fstream>
#include "Scepter_api.h"
#include <thread>

using namespace std;
ScDeviceInfo* pDeviceListInfo = NULL;
ScDeviceHandle deviceHandle = 0;

bool InitDevice(const int deviceCount);
void HotPlugStateCallback(const ScDeviceInfo *pInfo, int status, void *contex);

int main(int argc, char *argv[])
{
	cout << "---DevHotPlugCallbackC---" << endl;

	ScStatus status = scInitialize();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scInitialize] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scInitialize] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	uint32_t deviceCount = 0;
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

	if (InitDevice(deviceCount))
	{
		status = scSetHotPlugStatusCallback(HotPlugStateCallback, nullptr);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scSetHotPlugStatusCallback] success, ScStatus(" << status << "). Waiting for hotplug operation." << endl;
			for (;;)
			{
				this_thread::sleep_for(chrono::seconds(1));
			}
		}
		else
		{
			cout << "[scSetHotPlugStatusCallback] fail, ScStatus(" << status << ")." << endl;
		}

		status = scCloseDevice(&deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scCloseDevice] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scCloseDevice] fail, ScStatus(" << status << ")." << endl;
		}
	}

	status = scShutdown();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scShutdown] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scShutdown] fail, ScStatus(" << status << ")." << endl;
	}

	if (pDeviceListInfo)
	{
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
	}

	return 0;
}

bool InitDevice(const int deviceCount)
{
	if (pDeviceListInfo)
	{
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
	}

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	ScStatus status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceInfoList] success, ScStatus(" << status << ").";
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << " The first device [status]: " << pDeviceListInfo[0].status << " does not support connection." << endl;
			return false;
		}
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		return false;
	}

	cout << " The first deviceInfo, <serialNumber>: " << pDeviceListInfo[0].serialNumber
		<< ", <ip>: " << pDeviceListInfo[0].ip << ", <status>: " << pDeviceListInfo[0].status << endl;

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scOpenDeviceBySN] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scOpenDeviceBySN] fail, ScStatus(" << status << ")." << endl;
		return false;
	}

	status = scStartStream(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStartStream] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStartStream] fail, ScStatus(" << status << ")." << endl;
		return false;
	}

	return true;
}

void HotPlugStateCallback(const ScDeviceInfo *pInfo, int status, void *contex)
{
	if (status == 0)
	{
		cout << endl << "The device is <Added>, deviceInfo <serialNumber>: " << pInfo->serialNumber
			<< ", <ip>: " << pInfo->ip << ", <status>: " << pInfo->status << endl;

		ScStatus status = scOpenDeviceBySN(pInfo->serialNumber, &deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scOpenDeviceBySN] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scOpenDeviceBySN] fail, ScStatus(" << status << ")." << endl;
			return;
		}

		status = scStartStream(deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStartStream] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scStartStream] fail, ScStatus(" << status << ")." << endl;
			return;
		}
	}
	else
	{
		cout << endl << "The device is <Removed>, deviceInfo <serialNumber>: " << pDeviceListInfo[0].serialNumber
			<< ", <ip>: " << pDeviceListInfo[0].ip << ", <status>: " << pDeviceListInfo[0].status << endl;

		ScStatus status = scStopStream(deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStopStream] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scStopStream] fail, ScStatus(" << status << ")." << endl;
			return;
		}

		status = scCloseDevice(&deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scCloseDevice] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scCloseDevice] fail, ScStatus(" << status << ")." << endl;
			return;
		}
	}
}
