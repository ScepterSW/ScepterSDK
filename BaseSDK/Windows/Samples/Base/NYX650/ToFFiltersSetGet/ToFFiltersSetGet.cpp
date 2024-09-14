#include <thread>
#include <iostream>
#include "Scepter_api.h"
using namespace std;

int main()
{
	cout << "--------------ToFFiltersSetGet-------------"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
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
		return false;
	}

    cout << "scOpenDeviceBySN status :" << status << endl;

	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" <<status<< endl;
		return -1;
	}

	//The parameters of TimeFilter and ConfidenceFilter are stored in camera
	//The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK

	cout << endl << "-------------1------------ test TimeFilter --------------------------" << endl;

	ScTimeFilterParams TimeFilterParams = { 1,true };
	status = scGetTimeFilterParams(deviceHandle, &TimeFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetTimeFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default TimeFilter switch is " << boolalpha << TimeFilterParams.enable << endl;

	TimeFilterParams.enable = !TimeFilterParams.enable;
 	status = scSetTimeFilterParams(deviceHandle, TimeFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetTimeFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set TimeFilter switch to " << boolalpha << TimeFilterParams.enable <<" is Ok."<< endl;

	cout << endl << "-------------2--------- test ConfidenceFilter -----------------------" << endl;

	ScConfidenceFilterParams confidenceFilterParams = { 15, true };
	status = scGetConfidenceFilterParams(deviceHandle, &confidenceFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default ConfidenceFilter switch is " << boolalpha << confidenceFilterParams.enable << endl;
	
	confidenceFilterParams.enable = !confidenceFilterParams.enable;
	status = scSetConfidenceFilterParams(deviceHandle, confidenceFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set ConfidenceFilter switch to " << boolalpha << confidenceFilterParams.enable << " is Ok." << endl;


	cout << endl << "-------------3--------- test FlyingPixelFilter ----------------------" << endl;

	ScFlyingPixelFilterParams flyingPixelFilterParams = { 15, true };
	status = scGetFlyingPixelFilterParams(deviceHandle, &flyingPixelFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFlyingPixelFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default FlyingPixelFilter switch is " << boolalpha << flyingPixelFilterParams.enable << endl;
	
	flyingPixelFilterParams.enable = !flyingPixelFilterParams.enable;
	status = scSetFlyingPixelFilterParams(deviceHandle,flyingPixelFilterParams);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetFlyingPixelFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set FlyingPixelFilter switch to " << boolalpha << flyingPixelFilterParams.enable << " is Ok." << endl;
	
	cout << endl << "-------------4---------- test FillHoleFilter ------------------------" << endl;

	bool bFillHoleFilter= true;
	status = scGetFillHoleFilterEnabled(deviceHandle, &bFillHoleFilter);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFillHoleFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "The default FillHoleFilter switch is " << boolalpha << bFillHoleFilter << endl;
	
	bFillHoleFilter = !bFillHoleFilter;
	status = scSetFillHoleFilterEnabled(deviceHandle, bFillHoleFilter);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetFillHoleFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "Set FillHoleFilter switch to " << boolalpha << bFillHoleFilter << " is Ok." << endl;

	cout << endl << "-------------5---------- test SpatialFilter -------------------------" << endl;

	bool bSpatialFilter = true;
	status = scGetSpatialFilterEnabled(deviceHandle, &bSpatialFilter);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetSpatialFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "The default SpatialFilter switch is " << boolalpha << bSpatialFilter << endl;
	
	bSpatialFilter = !bSpatialFilter;
	status = scSetSpatialFilterEnabled(deviceHandle, bSpatialFilter);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetSpatialFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "Set SpatialFilter switch to " << boolalpha << bSpatialFilter << " is Ok." << endl;

	//Stop capturing the image stream
	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" <<status<< endl;
		return -1;
	}

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

	cout << endl << "---Test end, please reboot camera to restore the default settings.----" << endl;
	return 0;
}
