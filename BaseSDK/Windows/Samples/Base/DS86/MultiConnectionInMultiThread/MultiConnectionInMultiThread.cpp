#include <thread>
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "Scepter_api.h"

#define frameSpace 10
using namespace std;

class Device
{
public:
	void TestDevice(ScDeviceInfo * pDeviceListInfo)
	{
		lock_guard<mutex> lk(mutex_);
		cout << "The deviceInfo, <serialNumber>: " << pDeviceListInfo->serialNumber
			<< ", <ip>: " << pDeviceListInfo->ip << ", <status>: " << pDeviceListInfo->status << endl;

		ScStatus status = scOpenDeviceBySN(pDeviceListInfo->serialNumber, &device_);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scOpenDeviceBySN] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}
		else
		{
			cout << "[scOpenDeviceBySN] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
			isTestDone_ = true;
			condition_.notify_one();
			return;
		}

		status = scStartStream(device_);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStartStream] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}
		else
		{
			cout << "[scStartStream] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
			isTestDone_ = true;
			condition_.notify_one();
			return;
		}

		//Wait for the device to upload image data.
		this_thread::sleep_for(chrono::milliseconds(1000));

		ScFrame depthFrame = { 0 };
		ScFrameReady frameReady = { 0 };
		for (int j = 0; j < frameSpace; j++)
		{
			status = scGetFrameReady(device_, 1200, &frameReady);
			if (status == ScStatus::SC_OK)
			{
				cout << "[scGetFrameReady] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
			}
			else
			{
				cout << "[scGetFrameReady] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
				continue;
			}

			if (1 == frameReady.depth)
			{
				status = scGetFrame(device_, SC_DEPTH_FRAME, &depthFrame);
				if (status == ScStatus::SC_OK)
				{
					cout << "[scGetFrame] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << ", <frameIndex>: " << depthFrame.frameIndex << endl;
				}
				else
				{
					cout << "[scGetFrame] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
				}
			}
		}

		status = scStopStream(device_);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scStopStream] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}
		else
		{
			cout << "[scStopStream] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}

		status = scCloseDevice(&device_);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scCloseDevice] success, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}
		else
		{
			cout << "[scCloseDevice] fail, ScStatus(" << status << "). The device <serialNumber>:" << pDeviceListInfo->serialNumber << endl;
		}
		isTestDone_ = true;
		condition_.notify_one();

		return;
	}

	bool WaitForTestDone()
	{
		unique_lock<mutex> lk(mutex_);
		condition_.wait(lk, [this] {return isTestDone_; });
		return isTestDone_;
	}

private:
	ScDeviceHandle device_;
	bool isTestDone_ = false;
	mutex mutex_;
	condition_variable condition_;
};

int main()
{
	cout << "---MultiConnectionInMultiThread---" << endl;

	uint32_t deviceCount = 0;
	ScStatus status = SC_OTHERS;
	ScDeviceInfo* pDeviceListInfo = nullptr;

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

	do
	{
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
	} while (deviceCount < 2);

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceInfoList] success, ScStatus(" << status << ")." << endl;

		Device* pDevice = new Device[deviceCount];
		for (uint32_t i = 0; i < deviceCount; i++)
		{
			thread(&Device::TestDevice, &(pDevice[i]), &pDeviceListInfo[i]).detach();
		}

		//Waiting for thread exit.
		for (uint32_t i = 0; i < deviceCount; i++)
		{
			pDevice[i].WaitForTestDone();
		}
		delete[] pDevice;
		pDevice = NULL;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
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
