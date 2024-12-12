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

	uint32_t deviceCount = 0;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	ScFrameReady frameReady = { 0 };
	ScFrame depthFrame = { 0 };
	ofstream csvWriter;
	uint64_t endTimestamp = 0;
	uint64_t startTimestamp = 0;
	uint64_t deviceTimestamp = 0;
	uint64_t frameInterval = 0;
	uint64_t frameIntervalNTP = 0;

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

	status = scSetWorkMode(deviceHandle, SC_SOFTWARE_TRIGGER_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetWorkMode] success, ScStatus(" << status << "). Set SC_SOFTWARE_TRIGGER_MODE." << endl;
	}
	else
	{
		cout << "[scSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameRate] success, ScStatus(" << status << "). The device frame rate is " << frameRate << endl;
	}
	else
	{
		cout << "[scGetFrameRate] fail, ScStatus(" << status << ")." << endl;
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

	string fileName = "";
	csvWriter.open(fileName.append("SingleFrameDelayTest.csv"));
	if (!csvWriter.is_open())
	{
		cout << "csv file open failed" << endl;
		return -1;
	}
	csvWriter << "frameIndex,TotalDelay,ExcludeDelayofExposure" << endl;

	cout << "Please input the number of tests: ";
	uint32_t number = 0;
	cin >> number;

	for (int i = 0; i < number; i++)
	{
		timeb timeStart, timeEnd;
		status = scSoftwareTriggerOnce(deviceHandle);
		ftime(&timeStart);
		if (status == ScStatus::SC_OK)
		{
		}
		else
		{
			cout << "[scSoftwareTriggerOnce] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		//If the device is set with software trigger parameter, the ready time needs to be extended, so the setting here is 15000ms.
		status = scGetFrameReady(deviceHandle, 15000, &frameReady);
		if (status == ScStatus::SC_OK)
		{
		}
		else
		{
			cout << "[scGetFrameReady] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		if (1 == frameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (status == ScStatus::SC_OK)
			{
				if (depthFrame.frameIndex % 10 == 0)
				{
					cout << "SC_DEPTH_FRAME <frameIndex>: " << depthFrame.frameIndex << endl;
				}

				ftime(&timeEnd);
				endTimestamp = timeEnd.time * 1000 + timeEnd.millitm;
				startTimestamp = timeStart.time * 1000 + timeStart.millitm;
				deviceTimestamp = depthFrame.deviceTimestamp;
				frameInterval = endTimestamp - startTimestamp;
				frameIntervalNTP = endTimestamp - deviceTimestamp;

				//If the delay time is greater than 2000, it is considered that NTP is not enabled.
				if (frameIntervalNTP > 2000)
				{
					csvWriter << depthFrame.frameIndex << "," << frameInterval << ",N/A" << endl;
				}
				else
				{
					csvWriter << depthFrame.frameIndex << "," << frameInterval << "," << frameIntervalNTP << endl;
				}
			}
			else
			{
				cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
			}
		}

		//The time interval between two triggers should be greater than 
		//the time interval between the two frames generated.
		this_thread::sleep_for(chrono::milliseconds(1000 / frameRate));
	}
	cout << "Save delay time successful in " << fileName.c_str() << endl;
	csvWriter.close();

	status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetWorkMode] success, ScStatus(" << status << "). Set SC_ACTIVE_MODE." << endl;
	}
	else
	{
		cout << "[scSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return -1;
	}

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
	cout << "---End---" << endl;

	return 0;
}
