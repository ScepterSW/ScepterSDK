#include <thread>
#include <iostream>
#include "Scepter_api.h"

using namespace std;

int main()
{
	cout << "---DeviceParamSetGet---"<< endl;

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
		return -1;
	}

	//cameraParameters
	ScSensorIntrinsicParameters cameraParameters;
	status = scGetSensorIntrinsicParameters(deviceHandle, SC_TOF_SENSOR, &cameraParameters);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetCameraParameters failed status:" << status << endl;
		return -1;
	}
	cout << endl;
	cout << "Get ScGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic:" << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;
	cout << endl;

	//gmmgain
	uint8_t gmmgain = 0;
	status = scGetIRGMMGain(deviceHandle, &gmmgain);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetGMMGain failed status:" << status << endl;
		return -1;
	}
	cout << "default gmmgain: " << (int)gmmgain << endl;

	gmmgain = 50;
	status = scSetIRGMMGain(deviceHandle, gmmgain);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetGMMGain failed status:" << status << endl;
		return -1;
	}
	status = scGetIRGMMGain(deviceHandle, &gmmgain);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetGMMGain failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "set gmmgain: " << (int)gmmgain << " succeeded" << endl;
	}

	cout << endl;

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

	cout << "--end--"<< endl;
	return 0;
}
