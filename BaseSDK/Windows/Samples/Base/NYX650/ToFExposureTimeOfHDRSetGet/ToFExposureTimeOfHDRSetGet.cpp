#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---ToFExposureTimeOfHDRSetGet---" << endl;

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

	int32_t frameRate = 5;
	status = scSetFrameRate(deviceHandle, frameRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetFrameRate] success, ScStatus(" << status << "). Set the device frame rate to " << frameRate << endl;
	}
	else
	{
		cout << "[scSetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	bool enabled = false;
	status = scGetHDRModeEnabled(deviceHandle, &enabled);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetHDRModeEnabled] success, ScStatus(" << status << "). Get HDR mode status: " << boolalpha << enabled << endl;
	}
	else
	{
		cout << "[scGetHDRModeEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	//If HDR is disabled, enable it.
	if (!enabled)
	{
		ScExposureControlMode eControlMode;
		status = scGetExposureControlMode(deviceHandle, SC_TOF_SENSOR, &eControlMode);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scGetExposureControlMode] success, ScStatus(" << status << "). Get exposure control mode: " <<
				(SC_EXPOSURE_CONTROL_MODE_AUTO == eControlMode ? "SC_EXPOSURE_CONTROL_MODE_AUTO" : "SC_EXPOSURE_CONTROL_MODE_MANUAL") << endl;
		}
		else
		{
			cout << "[scGetExposureControlMode] fail, ScStatus(" << status << ")." << endl;
			return -1;
		}

		if (SC_EXPOSURE_CONTROL_MODE_AUTO == eControlMode)
		{
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
		}

		//Set HDR mode enabled, WDR function need to be turned off in advance.
		status = scSetHDRModeEnabled(deviceHandle, true);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scSetHDRModeEnabled] success, ScStatus(" << status << "). Enabled HDR mode." << endl;
		}
		else
		{
			cout << "[scSetHDRModeEnabled] fail, ScStatus(" << status << ")." << endl;
			return -1;
		}
	}

	int32_t nCount = 0;
	status = scGetFrameCountOfHDRMode(deviceHandle, &nCount);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameCountOfHDRMode] success, ScStatus(" << status << "). Get HDR mode fame count: " << nCount << endl;
	}
	else
	{
		cout << "[scGetFrameCountOfHDRMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	for (int i = 0; i < nCount; i++)
	{
		int32_t maxExposureTime = 0;
		status = scGetMaxExposureTimeOfHDR(deviceHandle, i, &maxExposureTime);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scGetMaxExposureTimeOfHDR] success, ScStatus(" << status << "). Get frame count: " << i << " max HDR exposure time: " << maxExposureTime << endl;
		}
		else
		{
			cout << "[scGetMaxExposureTimeOfHDR] fail, ScStatus(" << status << ")." << endl;
			return -1;
		}

		int32_t curExposureTime = 0;
		status = scGetExposureTimeOfHDR(deviceHandle, i, &curExposureTime);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scGetExposureTimeOfHDR] success, ScStatus(" << status << "). Get frame count: " << i << " current HDR exposure time: " << curExposureTime << endl;
		}
		else
		{
			cout << "[scGetExposureTimeOfHDR] fail, ScStatus(" << status << ")." << endl;
			return -1;
		}

		int32_t exposureTime = maxExposureTime / 2;
		status = scSetExposureTimeOfHDR(deviceHandle, i, exposureTime);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scSetExposureTimeOfHDR] success, ScStatus(" << status << "). Set frame count: " << i << " HDR exposure time: " << exposureTime << endl;
		}
		else
		{
			cout << "[scSetExposureTimeOfHDR] fail, ScStatus(" << status << ")." << endl;
			return -1;
		}
	}

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
