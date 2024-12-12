#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---IRGMMCorrectionSetGet---" << endl;

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

	ScIRGMMCorrectionParams IRGMMCorrectionParams = { 1, true };
	status = scGetIRGMMCorrection(deviceHandle, &IRGMMCorrectionParams);
	if (status == ScStatus::SC_OK)
	{
		if (IRGMMCorrectionParams.enable)
		{
			cout << "[scGetIRGMMCorrection] success, ScStatus(" << status << "). The device default IR GMM correction enable is " << boolalpha << IRGMMCorrectionParams.enable << ", threshold is " << IRGMMCorrectionParams.threshold << endl;
		}
		else
		{
			cout << "[scGetIRGMMCorrection] success, ScStatus(" << status << "). The device default IR GMM correction enable is " << boolalpha << IRGMMCorrectionParams.enable << endl;
		}
	}
	else
	{
		cout << "[scGetIRGMMCorrection] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	IRGMMCorrectionParams.enable = !IRGMMCorrectionParams.enable;
	if (IRGMMCorrectionParams.enable)
	{
		IRGMMCorrectionParams.threshold = IRGMMCorrectionParams.threshold / 2 + 30;
	}
	status = scSetIRGMMCorrection(deviceHandle, IRGMMCorrectionParams);
	if (status == ScStatus::SC_OK)
	{
		if (IRGMMCorrectionParams.enable)
		{
			cout << "[scSetIRGMMCorrection] success, ScStatus(" << status << "). Set the device IR GMM correction enable is " << boolalpha << IRGMMCorrectionParams.enable << ", threshold is " << IRGMMCorrectionParams.threshold << endl;
		}
		else
		{
			cout << "[scSetIRGMMCorrection] success, ScStatus(" << status << "). Set the device IR GMM correction enable is " << boolalpha << IRGMMCorrectionParams.enable << endl;
		}
	}
	else
	{
		cout << "[scSetIRGMMCorrection] fail, ScStatus(" << status << ")." << endl;
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
	cout << "---Please reboot camera to restore the default settings---" << endl;
	cout << "---End---" << endl;

	return 0;
}
