#include <thread>
#include <iostream>
#include "Scepter_api.h"
#define frameSpace 20

using namespace std;

int main()
{
	cout << "---DeviceHWTriggerMode---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	//about frame
	
	ScFrameReady FrameReady = { 0 };
	ScFrame depthFrame = { 0 };
	bool bSlaveEnabled = true;

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
	
	//set slave true
	status = scSetWorkMode(deviceHandle, SC_HARDWARE_TRIGGER_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return -1;
	}

	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}

    cout << "Please trigger the hardware signal to start the hardware trigger test" << endl;

	//1.hardware trigger.
	//2.ReadNextFrame.
	//3.GetFrame acoording to Ready flag and Frametype.
	for(int i = 0;i < frameSpace; i++)
	{
        //The minimum time interval to trigger a signal is 1000/FPS milliseconds
		//Wait for an external trigger signal. If the external signal is triggered once, the device sends a frame
		//If no image is ready within 1200ms, the function will return ScRetGetFrameReadyTimeOut
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" <<status<<endl;
			continue;
		}

		//depthFrame for example.
		if (1 == FrameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				cout << "scGetFrame status:" << status << "  "
				<< "frameType:" << depthFrame.frameType << "  "
				<< "frameIndex:" << depthFrame.frameIndex << endl;
			}
		}

	}
	
	//set slave false
	status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return -1;
	}
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
		cout << "scCloseDevice failed status:" << status << endl;
		return -1;
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" << status << endl;
		return -1;
	}
	cout << "---end---"<< endl;

	return 0;
}
