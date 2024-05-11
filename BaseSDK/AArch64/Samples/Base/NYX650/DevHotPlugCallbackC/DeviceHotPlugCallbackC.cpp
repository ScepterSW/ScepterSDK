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
	uint32_t deviceCount = 0;

	ScStatus status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "ScInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	status = scGetDeviceCount(&deviceCount, 3000);
	if (status != ScStatus::SC_OK)
	{
		cout << "ScGetDeviceCount failed status:" <<status << endl;
		system("pause");
		return -1;
	}
	cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		cout << "scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples."<< endl;
		system("pause");
		return -1;
	}

	if (InitDevice(deviceCount))
	{
		status = scSetHotPlugStatusCallback(HotPlugStateCallback, nullptr);;
		if (status != ScStatus::SC_OK)
		{
			cout << "SetHotPlugStatusCallback failed status:" <<status << endl;
 		}
		else
		{
			cout <<" wait for hotplug operation "<<endl;
			// wait for hotplug
			for (;;)
			{	
				this_thread::sleep_for(chrono::seconds(1));
			}
		}
		status = scCloseDevice(&deviceHandle);
		if (status != ScStatus::SC_OK)
		{
			cout << "CloseDevice failed status:" <<status << endl;
		}
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "Shutdown failed status:" <<status << endl;
	} 
 
	delete[] pDeviceListInfo;
	pDeviceListInfo = NULL;

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
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection." << endl;
			return false;
		}
	}
	else
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		return false;
	}
	
	cout << "serialNumber:" << pDeviceListInfo[0].serialNumber << endl
	<< "ip:" << pDeviceListInfo[0].ip << endl
	<< "connectStatus:" << pDeviceListInfo[0].status << endl;

	deviceHandle = 0;

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);

	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return false;
	}

    cout << "open device successful,status :" << status << endl;

	status = scStartStream(deviceHandle);

	if (status != ScStatus::SC_OK)
	{
		cout << "StartStream failed status:" <<status << endl;
		return false;
	}

	return true;
}

void HotPlugStateCallback(const ScDeviceInfo *pInfo, int status, void *contex)
{
	cout << "ip " << status << "  " << pInfo->ip << "    " << (status == 0 ? "add" : "remove") << endl;
	cout << "serialNumber " << status << "  " << pInfo->serialNumber << "    " << (status == 0 ? "add" : "remove") << endl;

	if (status == 0)
	{
		cout << "scOpenDevice " << scOpenDeviceBySN(pInfo->serialNumber, &deviceHandle) << endl;
		cout << "scStartStream " << scStartStream(deviceHandle) << endl;
	}
	else
	{
		cout << "scStopStream " << scStopStream(deviceHandle) << endl;
		cout << "scCloseDevice " << scCloseDevice(&deviceHandle) << endl;
	}
}
