#pragma warning(disable:4996)

#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>
#include <sstream>
#define frameSpace 20

using namespace std;

int main()
{
	cout << "---FrameCaptureAndSave---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	
	//about frame
	
	ScFrameReady FrameReady = {0};
	ScFrame depthFrame = {0};

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "ScInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
	status = scGetDeviceCount(&deviceCount, 3000);
	if (status != ScStatus::SC_OK)
	{
		cout << "ScGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
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

	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" <<status<< endl;
		return -1;
	}

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//1.ReadNextFrame.
	//2.GetFrame acoording to Ready flag and Frametype.
	for(int i = 0;i < frameSpace;i++)
	{
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" <<status<< endl;
			continue;
		}

		//depthFrame for example.
		if(1 == FrameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				cout << "get Frame successful,status:" << status << "  ";

				//name of the frames to be saved.
				stringstream framename;
				framename << "depthFrame"<< depthFrame.frameIndex<< ".bin";
				char fname[64];
				framename >> fname;

				//save
				ofstream file;
				file.open(fname, ios::binary|ios::trunc);
				file.write((char*)depthFrame.pFrameData, depthFrame.dataLen);
				file.close();

				std::cout << "Save..." << std::endl;
				break;	
			}
		}
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
	cout << "---end---"<< endl;

	return 0;
}
