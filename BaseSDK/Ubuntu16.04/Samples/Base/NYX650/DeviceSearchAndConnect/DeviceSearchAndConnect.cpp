#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---DeviceSearchAndConnect---"<< endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "scInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
	status = scGetDeviceCount(&deviceCount, 3000);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
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

	pDeviceInfo = new ScDeviceInfo;
	deviceCount = 1;
	status = scGetDeviceInfoList(deviceCount, pDeviceInfo);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetDeviceInfoList failed status:" <<status<< endl;
		return -1;
	}

	cout << "serialNumber:" << pDeviceInfo->serialNumber << endl
		<< "ip:" << pDeviceInfo->ip << endl
		<< "connectStatus:" << pDeviceInfo->status << endl;

	status = scOpenDeviceBySN(pDeviceInfo->serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return -1;
	}
	cout << "scOpenDeviceBySN status :" << status << endl;

	//1.close device
	//2.SDK shutdown
	status = scCloseDevice(&deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scCloseDevice failed status:" << status << endl;
		return -1;
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" << status << endl;
		return -1;
	}
	cout << "--end--"<< endl;

	return 0;
}
