#include <thread>
#include <iostream>
#include "Scepter_api.h"
using namespace std;

int main()
{
	cout << "---- ToFExposureTimeSetGet ----"<< endl;

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
	//2.Get infomation of the devices. 
	//3.Open devices accroding to the info.
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

	//1.Default FrameRate
	//2.Set new ExposureTime
	//3.Change FrameRate to 5 (The exposure time ranges are different at different frame rates)
	//4.Set new ExposureTime

	cout << endl << "---- Default FrameRate ----" << endl;
	//Set Control mode to manual
	status = scSetExposureControlMode(deviceHandle, SC_TOF_SENSOR, SC_EXPOSURE_CONTROL_MODE_MANUAL);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureControlMode failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "Set control mode to manual." << endl;
	}

	//Get default frame rate
	int defaultframeRate = 10; 
	status = scGetFrameRate(deviceHandle, &defaultframeRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "Get default frame rate: " << defaultframeRate  << endl;

	//Get the range of the ToF exposure time 
	int maxExposureTime = 0;
	status = scGetMaxExposureTime(deviceHandle, SC_TOF_SENSOR, &maxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	cout <<"Recommended scope: 58 - " << maxExposureTime << endl;

	//Set new ExposureTime
	int exposureTime = 400;
	status = scSetExposureTime(deviceHandle, SC_TOF_SENSOR, exposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "Set exposure time "<< exposureTime << " is OK." << endl;
	}

	cout << endl << "---- Set FrameRate to 5 ----" << endl;
	//Set new FrameRate
	int frameRate = 5;  //New frame rate, can change it
	status = scSetFrameRate(deviceHandle, frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetFrameRate set new FrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "Set frame rate " << frameRate << " is OK." << endl;

	//Need to get new ExposureTime Range due to FrameRate change.
	status = scGetMaxExposureTime(deviceHandle, SC_TOF_SENSOR, &maxExposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetMaxExposureTime failed status:" << status << endl;
		return -1;
	}
	cout << "recommended scope: 58 - " << maxExposureTime << endl;

	//Set new ExposureTime 500	
	exposureTime = 500;
 	status = scSetExposureTime(deviceHandle, SC_TOF_SENSOR, exposureTime);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetExposureTime failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "Set exposure time " << exposureTime << " is OK." << endl;
	}

	//Stop capturing the image stream
	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" <<status<< endl;
		return -1;
	}

	//1.Close device
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

	cout << endl << "--- Test end, please reboot camera to restore the default settings ---" << endl;
	return 0;
}
