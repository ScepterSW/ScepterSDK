#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---ToFExposureTimeSetGet---" << endl;

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

	//Testing TOF exposure time requires turning off HDR and WDR in advance.
	cout << endl << "---1. Default frame rate---" << endl;
	status = scSetExposureControlMode(deviceHandle, SC_TOF_SENSOR, SC_EXPOSURE_CONTROL_MODE_MANUAL);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureControlMode] success, ScStatus(" << status << "). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success." << endl;
	}
	else
	{
		cout << "[scSetExposureControlMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int defaultframeRate = 10;
	status = scGetFrameRate(deviceHandle, &defaultframeRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameRate] success, ScStatus(" << status << "). The device frame rate is " << defaultframeRate << endl;
	}
	else
	{
		cout << "[scGetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int maxExposureTime = 0;
	status = scGetMaxExposureTime(deviceHandle, SC_TOF_SENSOR, &maxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetMaxExposureTime] success, ScStatus(" << status << "). Recommended scope: 58 - " << maxExposureTime << endl;
	}
	else
	{
		cout << "[scGetMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int exposureTime = 400;
	status = scSetExposureTime(deviceHandle, SC_TOF_SENSOR, exposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureTime] success, ScStatus(" << status << "). Set exposure time " << exposureTime << " is OK." << endl;
	}
	else
	{
		cout << "[scSetExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---2. Set frame rate to 5---" << endl;
	int frameRate = 5;
	status = scSetFrameRate(deviceHandle, frameRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetFrameRate] success, ScStatus(" << status << "). Set frame rate " << frameRate << " is OK." << endl;
	}
	else
	{
		cout << "[scSetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scGetMaxExposureTime(deviceHandle, SC_TOF_SENSOR, &maxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetMaxExposureTime] success, ScStatus(" << status << "). Recommended scope: 58 - " << maxExposureTime << endl;
	}
	else
	{
		cout << "[scGetMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	exposureTime = 500;
	status = scSetExposureTime(deviceHandle, SC_TOF_SENSOR, exposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureTime] success, ScStatus(" << status << "). Set exposure time " << exposureTime << " is OK." << endl;
	}
	else
	{
		cout << "[scSetExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}
	cout << endl;

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
	cout << "---Please reboot camera to restore the default settings---" << endl;
	cout << "---End---" << endl;

	return 0;
}
