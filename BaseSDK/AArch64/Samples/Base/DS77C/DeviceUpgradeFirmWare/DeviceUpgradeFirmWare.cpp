#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---DeviceUpgradeFirmWare---" << endl;

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

	cout << "Please input firmware file path:";
	char pImgPath[256];
	cin.getline(pImgPath, 256);

	status = scStartUpgradeFirmWare(deviceHandle, pImgPath);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStartUpgradeFirmWare] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStartUpgradeFirmWare] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int upgradeStatus = 0;
	int process = 0;
	while (true)
	{
		status = scGetUpgradeStatus(deviceHandle, &upgradeStatus, &process);
		if (status == SC_OK)
		{
			cout << "[scGetUpgradeStatus] success, ScStatus(" << status << ").";
			cout << " Upgrade firmware status:" << upgradeStatus << ", process:" << process << endl;
			if (upgradeStatus != 0)
			{
				cout << "upgrade failed." << endl;
				break;
			}
			else
			{
				//Upgrade progress is 100, upgrade successful. After the upgrade is successful, 
				//the SDK will automatically reboot the device internally to make the upgrade file effective.
				if (process == 100)
				{
					cout << "Upgrade done." << endl;
					cout << "Waiting for reboot." << endl;
					this_thread::sleep_for(chrono::seconds(10));
					cout << "Reboot done." << endl;
					break;
				}
			}
		}
		else
		{
			cout << "[scGetUpgradeStatus] fail, ScStatus(" << status << ")." << endl;
			break;
		}
		this_thread::sleep_for(chrono::milliseconds(1000));
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
