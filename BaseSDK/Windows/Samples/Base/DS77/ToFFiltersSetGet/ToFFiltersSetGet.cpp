#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---ToFFiltersSetGet---" << endl;

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

	//The parameters of TimeFilter and ConfidenceFilter are stored in camera 
	//and will become invalid if the device is powered off.
	//The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK 
	//and will become invalid if the program exit.
	cout << endl << "---1. Test TimeFilter---" << endl;
	ScTimeFilterParams TimeFilterParams = { 1,true };
	status = scGetTimeFilterParams(deviceHandle, &TimeFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetTimeFilterParams] success, ScStatus(" << status << "). The default time filter switch is " << boolalpha << TimeFilterParams.enable << endl;
	}
	else
	{
		cout << "[scGetTimeFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	TimeFilterParams.enable = !TimeFilterParams.enable;
	status = scSetTimeFilterParams(deviceHandle, TimeFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetTimeFilterParams] success, ScStatus(" << status << "). Set time filter switch to " << boolalpha << TimeFilterParams.enable << " is Ok." << endl;
	}
	else
	{
		cout << "[scSetTimeFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---2. Test ConfidenceFilter---" << endl;
	ScConfidenceFilterParams confidenceFilterParams = { 15, true };
	status = scGetConfidenceFilterParams(deviceHandle, &confidenceFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetConfidenceFilterParams] success, ScStatus(" << status << "). The default confidence filter switch is " << boolalpha << confidenceFilterParams.enable << endl;
	}
	else
	{
		cout << "[scGetConfidenceFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	confidenceFilterParams.enable = !confidenceFilterParams.enable;
	status = scSetConfidenceFilterParams(deviceHandle, confidenceFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetConfidenceFilterParams] success, ScStatus(" << status << "). Set confidence filter switch to " << boolalpha << confidenceFilterParams.enable << " is Ok." << endl;
	}
	else
	{
		cout << "[scSetConfidenceFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---3. Test FlyingPixelFilter---" << endl;
	ScFlyingPixelFilterParams flyingPixelFilterParams = { 15, true };
	status = scGetFlyingPixelFilterParams(deviceHandle, &flyingPixelFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFlyingPixelFilterParams] success, ScStatus(" << status << "). The default flying pixel filter switch is " << boolalpha << flyingPixelFilterParams.enable << endl;
	}
	else
	{
		cout << "[scGetFlyingPixelFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	flyingPixelFilterParams.enable = !flyingPixelFilterParams.enable;
	status = scSetFlyingPixelFilterParams(deviceHandle, flyingPixelFilterParams);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetFlyingPixelFilterParams] success, ScStatus(" << status << "). Set flying pixel filter switch to " << boolalpha << flyingPixelFilterParams.enable << " is Ok." << endl;
	}
	else
	{
		cout << "[scSetFlyingPixelFilterParams] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---4. Test FillHoleFilter---" << endl;
	bool bFillHoleFilter = true;
	status = scGetFillHoleFilterEnabled(deviceHandle, &bFillHoleFilter);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFillHoleFilterEnabled] success, ScStatus(" << status << "). The default fill hole filter switch is " << boolalpha << bFillHoleFilter << endl;
	}
	else
	{
		cout << "[scGetFillHoleFilterEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	bFillHoleFilter = !bFillHoleFilter;
	status = scSetFillHoleFilterEnabled(deviceHandle, bFillHoleFilter);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetFillHoleFilterEnabled] success, ScStatus(" << status << "). Set fill hole filter switch to " << boolalpha << bFillHoleFilter << " is Ok." << endl;
	}
	else
	{
		cout << "[scSetFillHoleFilterEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	cout << endl << "---5. Test SpatialFilter---" << endl;
	bool bSpatialFilter = true;
	status = scGetSpatialFilterEnabled(deviceHandle, &bSpatialFilter);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetSpatialFilterEnabled] success, ScStatus(" << status << "). The default spatial filter switch is " << boolalpha << bSpatialFilter << endl;
	}
	else
	{
		cout << "[scGetSpatialFilterEnabled] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	bSpatialFilter = !bSpatialFilter;
	status = scSetSpatialFilterEnabled(deviceHandle, bSpatialFilter);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetSpatialFilterEnabled] success, ScStatus(" << status << "). Set spatial filter switch to " << boolalpha << bSpatialFilter << " is Ok." << endl;
	}
	else
	{
		cout << "[scSetSpatialFilterEnabled] fail, ScStatus(" << status << ")." << endl;
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
