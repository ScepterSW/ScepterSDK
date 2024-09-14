#include <iostream>
#include <sys/timeb.h>
#include <thread>
#include <fstream>
#include <string>
#include "Scepter_api.h"
using namespace std;
int main()
{
	cout << "---SingleFrameDelayTest---" << endl;
	ScStatus status = SC_OTHERS;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "scInitialize failed status:" << status << endl;
		system("pause");
		return -1;
	}

	uint32_t deviceCount = 0;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
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
		cout << "scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples." << endl;
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
			cout << "The device state does not support connection." << endl;
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
		cout << "OpenDevice failed status:" << status << endl;
		return false;
	}
	// set soft work model
	status = scSetWorkMode(deviceHandle, SC_SOFTWARE_TRIGGER_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return -1;
	}
	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "scGetFrameRate : " << frameRate << endl;
	}
	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}
	//Wait for the device to upload image data
	ScFrameReady FrameReady = { 0 };
	ScFrame depthFrame = { 0 };
	ofstream csvWriter;
	string fileName = "";
	csvWriter.open(fileName.append("SingleFrameDelayTest.csv"));
	if (!csvWriter.is_open())
	{
		cout << "csv file open failed"<< endl;
		return -1;
	}
	csvWriter << "frameIndex,TotalDelay,ExcludeDelayofExposure" << endl;
	//1.software trigger.
	//2.ReadNextFrame.
	//3.GetFrame acoording to Ready flag and Frametype.
	//4.sleep 1000/frameRate (ms)
	uint64_t endTimestamp = 0;
	uint64_t startTimestamp = 0;
	uint64_t deviceTimestamp = 0;
	uint64_t frameInterval = 0;
	uint64_t frameIntervalNTP = 0;

	cout << "Please input the number of tests:" << endl;
	uint32_t number = 0;
	cin >> number;
	for (int i = 0; i < number; i++)
	{
		timeb timeStart, timeEnd;
		//call the below api to trigger one frame, then the frame will be sent
		// if do not call this function, the frame will not be sent and the below call will return timeout fail
		status = scSoftwareTriggerOnce(deviceHandle);
		ftime(&timeStart); // record the start timestamp
		if (status != ScStatus::SC_OK)
		{
			cout << "scSoftwareTriggerOnce failed status:" << status << endl;
			continue;
		}
		//If no image is ready within 1200ms, the function will return ScRetGetFrameReadyTimeOut
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" << status << endl;
			continue;
		}
		if (1 == FrameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				if (depthFrame.frameIndex % 10 == 0)
				{
					cout << "scGetFrame,status:" << status << "  "
						<< "frameType:" << depthFrame.frameType << "  "
						<< "frameIndex:" << depthFrame.frameIndex << endl;
				}
				ftime(&timeEnd);// record the end timestamp
				endTimestamp = timeEnd.time * 1000 + timeEnd.millitm;
				startTimestamp = timeStart.time * 1000 + timeStart.millitm;
				deviceTimestamp = depthFrame.deviceTimestamp;
				frameInterval = endTimestamp - startTimestamp;
				frameIntervalNTP = endTimestamp - deviceTimestamp;

				if (frameIntervalNTP > 2000)
				{
					csvWriter << depthFrame.frameIndex << "," << frameInterval << ",N/A" << endl;
				}
				else
				{
					csvWriter<< depthFrame.frameIndex  << "," << frameInterval << "," << frameIntervalNTP <<endl;
				}
			}
		}
		this_thread::sleep_for(chrono::milliseconds(1000 / frameRate));
	}
	csvWriter.close();

	//set slave false
	status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return -1;
	}

	//Stop capturing the image stream
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
	cout << "---end---" << endl;

	return 0;
}
