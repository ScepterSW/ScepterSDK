#include <thread>
#include <iostream>
#include "Scepter_api.h"
#define frameSpace 10
using namespace std;

int main()
{
	cout << "---MultiConnection---"<< endl;

	//about dev
	uint32_t deviceCount = 0;
	ScStatus status = SC_OTHERS;
	ScDeviceHandle *deviceHandle = nullptr;
	ScDeviceInfo* pDeviceListInfo = nullptr;

	
	//about Frame
	
	ScFrameReady FrameReady = { 0 };
	ScFrame Depth = { 0 };

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
	do
	{
		status = scGetDeviceCount(&deviceCount, 3000);
		if (status != ScStatus::SC_OK)
		{
			cout << "ScGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
			system("pause");
			return -1;
		}
		cout << "Get device count: " << deviceCount << endl;
	} while (deviceCount < 2);

	deviceHandle = new ScDeviceHandle[deviceCount];
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
	for (int i = 0; i < deviceCount; i++)
	{
		cout << "CameraIndex: " << i << endl;
		cout << "serialNumber:" << pDeviceListInfo[i].serialNumber << endl
			<< "ip:" << pDeviceListInfo[i].ip << endl
			<< "connectStatus:" << pDeviceListInfo[i].status << endl;
	}
	for(int i = 0;i < deviceCount; i++)
	{
		status = scOpenDeviceBySN(pDeviceListInfo[i].serialNumber, &deviceHandle[i]);
		if (status != ScStatus::SC_OK)
		{
			cout << "OpenDevice failed status:" <<status << endl;
			return -1;
		}
	}

    for (int i = 0; i < deviceCount; i++)
    {
        //Starts capturing the image stream
        status = scStartStream(deviceHandle[i]);
        if (status != ScStatus::SC_OK)
        {
            cout << "scStartStream failed status:" << status << endl;
            return -1;
        }
    }

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//1.ReadNextFrame.
	//2.Get depth Frame acoording to Ready flag.
	for (int j = 0; j < frameSpace; j++)
	{
		for (int i = 0; i < deviceCount; i++)
		{
			if (deviceHandle[i])
			{
				ScFrame depthFrame = { 0 };
				ScFrameReady frameReady = { 0 };
				status = scGetFrameReady(deviceHandle[i], 1200, &frameReady);

				if (status != SC_OK)
				{
					cout << pDeviceListInfo[i].serialNumber <<"  scGetFrameReady failed status:" << status << endl;
					continue;
				}

				//Get depth frame, depth frame only output in following data mode
				if (1 == frameReady.depth)
				{
					status = scGetFrame(deviceHandle[i], SC_DEPTH_FRAME, &depthFrame);

					if (status == SC_OK && depthFrame.pFrameData != NULL)
					{
						cout << pDeviceListInfo[i].serialNumber << " frameIndex :" << depthFrame.frameIndex << endl;
					}
					else
					{
						cout << pDeviceListInfo[i].serialNumber << "scGetFrame SC_DEPTH_FRAME status:" << status  << endl;
					}
				}
			}

		}
	}
 
	//1.close device
	//2.SDK shutdown
	for (int i = 0; i < deviceCount; i++)
	{
		status = scStopStream(deviceHandle[i]);
		if (status != ScStatus::SC_OK)
		{
			cout << "scStopStream failed status:" <<status<< endl;
		}
		status = scCloseDevice(&deviceHandle[i]);
		if (status != ScStatus::SC_OK)
		{
			cout << "scCloseDevice failed status:" <<status<< endl;
		}
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
