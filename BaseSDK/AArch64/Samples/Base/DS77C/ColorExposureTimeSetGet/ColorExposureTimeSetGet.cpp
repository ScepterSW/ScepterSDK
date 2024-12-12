#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---ColorExposureTimeSetGet---" << endl;

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

	int rate = 10;
	status = scGetFrameRate(deviceHandle, &rate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameRate] success, ScStatus(" << status << "). The device frame rate is " << rate << endl;
	}
	else
	{
		cout << "[scGetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---To SC_EXPOSURE_CONTROL_MODE_MANUAL---" << endl;
	status = scSetExposureControlMode(deviceHandle, SC_COLOR_SENSOR, SC_EXPOSURE_CONTROL_MODE_MANUAL);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureControlMode] success, ScStatus(" << status << "). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success." << endl;
	}
	else
	{
		cout << "[scSetExposureControlMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << "---1. Get color sensor exposure time range with frame rate " << rate << "---" << endl;
	int maxExposureTime = 0;
	status = scGetMaxExposureTime(deviceHandle, SC_COLOR_SENSOR, &maxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetMaxExposureTime] success, ScStatus(" << status << "). Recommended scope: 100 - " << maxExposureTime << endl;
	}
	else
	{
		cout << "[scGetMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << "---2. Set and get color sensor new exposure time---" << endl;
	int exposureTime = 3000;
	status = scSetExposureTime(deviceHandle, SC_COLOR_SENSOR, exposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureTime] success, ScStatus(" << status << "). Set the device exposure time to " << exposureTime << endl;
	}
	else
	{
		cout << "[scSetExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scGetExposureTime(deviceHandle, SC_COLOR_SENSOR, &exposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetExposureTime] success, ScStatus(" << status << "). The device exposure time is " << exposureTime << endl;
	}
	else
	{
		cout << "[scGetExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---To SC_EXPOSURE_CONTROL_MODE_AUTO---" << endl;
	status = scSetExposureControlMode(deviceHandle, SC_COLOR_SENSOR, SC_EXPOSURE_CONTROL_MODE_AUTO);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetExposureControlMode] success, ScStatus(" << status << "). Set SC_EXPOSURE_CONTROL_MODE_AUTO success." << endl;
	}
	else
	{
		cout << "[scSetExposureControlMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << "---1. Get color exposure time range---" << endl;
	status = scGetMaxExposureTime(deviceHandle, SC_COLOR_SENSOR, &maxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetMaxExposureTime] success, ScStatus(" << status << "). Recommended scope: 100 - " << maxExposureTime << endl;
	}
	else
	{
		cout << "[scGetMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << "---2. Set and get color sensor new max exposure time range in auto mode---" << endl;
	int AECMaxExposureTime = 3000;
	status = scSetColorAECMaxExposureTime(deviceHandle, AECMaxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetColorAECMaxExposureTime] success, ScStatus(" << status << "). Set color AEC max exposure time to " << AECMaxExposureTime << endl;
	}
	else
	{
		cout << "[scSetColorAECMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	status = scGetColorAECMaxExposureTime(deviceHandle, &AECMaxExposureTime);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetColorAECMaxExposureTime] success, ScStatus(" << status << "). Get color AEC max exposure time is " << AECMaxExposureTime << endl;
	}
	else
	{
		cout << "[scGetColorAECMaxExposureTime] fail, ScStatus(" << status << ")." << endl;
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
	cout << "---End---" << endl;

	return 0;
}
