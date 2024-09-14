#include <thread>
#include <iostream>
#include "Scepter_api.h"
#define frameSpace 10
using namespace std;

int main()
{
	cout << "---ColorExposureTimeSetGet---"<< endl;

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
	//Get default frame rate
	int rate = 10;
	status = scGetFrameRate(deviceHandle, &rate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	// if need change the framerate, do first 
	/*
	rate = 5;
	scSetFrameRate(deviceHandle, rate);
	*/
	cout << endl << "---- To  SC_EXPOSURE_CONTROL_MODE_MANUAL ----" << endl;
	//switch exposure mode to manual
	status = scSetExposureControlMode(deviceHandle, SC_COLOR_SENSOR, SC_EXPOSURE_CONTROL_MODE_MANUAL);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureControlMode failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "scSetExposureControlMode  ok" << endl;
	}

	
	cout << "* step1. Get Color exposure time range with frameRate " << rate <<"*" << endl;
 	//Get the range of the Color exposure time 
	int maxExposureTime = 0;
	status = scGetMaxExposureTime(deviceHandle, SC_COLOR_SENSOR, &maxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	cout << "Recommended scope: 100 - " << maxExposureTime << endl;

	cout << "* step2. Set and Get new ExposureTime *" << endl;
	//Set new ExposureTime
	int exposureTime = 3000;
	status = scSetExposureTime(deviceHandle, SC_COLOR_SENSOR, exposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "SetExposureTime:"<< exposureTime << endl;
	}

	status = scGetExposureTime(deviceHandle, SC_COLOR_SENSOR, &exposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "GetExposureTime:"<< exposureTime << endl;
	}
		
	cout << endl << "---- To SC_EXPOSURE_CONTROL_MODE_AUTO ----" << endl;	
	//switch exposure mode to auto
	status = scSetExposureControlMode(deviceHandle, SC_COLOR_SENSOR, SC_EXPOSURE_CONTROL_MODE_AUTO);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureControlMode failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "scSetExposureControlMode ok" << endl;
	}
	
	cout << "* step1. Get Color exposure time range *" << endl;
	//Get the range of the Auto Color exposure time 
	status = scGetMaxExposureTime(deviceHandle, SC_COLOR_SENSOR, &maxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	cout << "Recommended scope: 100 - " << maxExposureTime << endl;

	cout << "* step2. Set and Get new Auto Max Color exposure time range *" << endl;
	//set new range of Auto Color exposure time. [100  maxExposureTime.exposureTime]
	int AECMaxExposureTime = 10000;
	status = scSetColorAECMaxExposureTime(deviceHandle, AECMaxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetColorAECMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "scSetColorAECMaxExposureTime:" << AECMaxExposureTime << endl;
	}

	//Get the new range of the Auto Color exposure time .
	status = scGetColorAECMaxExposureTime(deviceHandle, &AECMaxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetColorAECMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "scGetColorAECMaxExposureTime:" << AECMaxExposureTime << endl;
	}


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
	cout<< "---end---"<< endl;

	return 0;
}
