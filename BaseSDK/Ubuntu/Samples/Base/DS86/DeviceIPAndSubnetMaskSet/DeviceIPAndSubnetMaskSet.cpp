#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---DeviceIPAndSubnetMaskSet---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

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

	status = scSetDeviceDHCPEnabled(deviceHandle, false);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetDeviceDHCPEnabled] success, ScStatus(" << status << "). The device DHCP is close." << endl;
	}
	else
	{
		cout << "[scSetDeviceDHCPEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	string strIP = "192.168.1.102";
	status = scSetDeviceIPAddr(deviceHandle, strIP.c_str(), strIP.length());
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetDeviceIPAddr] success, ScStatus(" << status << "). Set the device IP to " << strIP.c_str() << endl;
	}
	else
	{
		cout << "[scSetDeviceIPAddr] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	string strSubMask = "255.255.255.0";
	status = scSetDeviceSubnetMask(deviceHandle, strSubMask.c_str(), strSubMask.length());
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetDeviceSubnetMask] success, ScStatus(" << status << "). Set the device subnet mask to " << strSubMask.c_str() << endl;
	}
	else
	{
		cout << "[scSetDeviceSubnetMask] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scRebootDevie(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scRebootDevie] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scRebootDevie] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << "Waiting for reboot." << endl;
	this_thread::sleep_for(chrono::seconds(10));
	cout << "Reboot done." << endl;

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
