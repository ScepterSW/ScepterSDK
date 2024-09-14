#include <thread>
#include <iostream>
#include <thread>
#include "Scepter_api.h"

using namespace std;

time_t GetTimeStampMS();

int main()
{
	cout << "---DeviceSetFrameRate---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "scInitialize failed status:" << status << endl;
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

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" << status << endl;
		return -1;
	}
    cout << "scOpenDeviceBySN status :" << status << endl;

	int frameRate = 5;
	status = scSetFrameRate(deviceHandle, frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetFrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "set frame rate :" << frameRate <<" is OK."<< endl;

	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}

    cout << "Start testing the average frame rate for 30 seconds, Please wait patiently" << endl;
	//statistical frame rate
	for (;;)
	{
		ScFrameReady FrameReady = {};
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{ 
			cout << "scGetFrameReady failed status:" << status << endl;
			continue;
		}

		if (1 == FrameReady.depth) 
		{
			ScFrame depthFrame = {};
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
                const int TESTPERIOD = 30;//30 senconds
				static int index = 0;
                static time_t start = GetTimeStampMS();

				time_t diff = GetTimeStampMS() - start;
                index++;
                if (diff > (TESTPERIOD * 1000))
                {
                    float fps = (index * TESTPERIOD * 1000.0f / diff) / TESTPERIOD;
                    index = 0;
					cout << fps << endl;
					break;
                }
			}
		}

	}

	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" << status << endl;
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

time_t GetTimeStampMS()
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    time_t timestamp = tp.time_since_epoch().count();
    return timestamp;
}