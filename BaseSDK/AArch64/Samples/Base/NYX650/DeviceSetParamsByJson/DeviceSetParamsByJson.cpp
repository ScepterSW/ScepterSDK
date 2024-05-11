#include <thread>
#include <iostream>
#include "Scepter_api.h"


using namespace std;

int main()
{
	cout << "---DeviceSetParamsByJson---"<< endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "ScInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
	status = scGetDeviceCount(&deviceCount, 3000);
	if (status != ScStatus::SC_OK)
	{
		cout << "ScGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
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

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection."<< endl;
			return -1;
		}
	}
	else
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		return -1;
	}

	cout << "serialNumber:" << pDeviceListInfo[0].serialNumber << endl
	<< "ip:" << pDeviceListInfo[0].ip << endl
	<< "connectStatus:" << pDeviceListInfo[0].status << endl;
	
	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return -1;
	}
	
	//Input Json file path

	cout << "Please input Json file path:" << endl;
	char pImgPath[256];
	cin.getline(pImgPath, 256);
	cout << pImgPath << endl;
	
	//Set the parameters by Json file
	//If fails, please refer to the log file of ScenseSDK.txt
	status = scSetParamsByJson(deviceHandle, pImgPath);
	cout << "scSetParamsByJson ret:" << status << endl;

	//1.close device
	//2.SDK shutdown
	status = scCloseDevice(&deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scCloseDevice failed status:" <<status<< endl;
		return -1;
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" <<status<< endl;
		return -1;
	}
	cout << "---end---"<< endl;

	return 0;
}
