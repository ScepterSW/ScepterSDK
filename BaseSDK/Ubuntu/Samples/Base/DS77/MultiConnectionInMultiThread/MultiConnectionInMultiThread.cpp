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
		cout << "serialNumber:" << pDeviceListInfo->serialNumber << endl
			 << "ip:" << pDeviceListInfo->ip << endl
			 << "connectStatus:" << pDeviceListInfo->status << endl;

		ScStatus status = scOpenDeviceBySN(pDeviceListInfo->serialNumber, &device_);
		if (status != ScStatus::SC_OK)
		{
			cout << "OpenDevice "<<pDeviceListInfo->serialNumber<<" failed status:" << status << endl;
			isTestDone_ = true;
			condition_.notify_one();
			return;
		}

		// Starts capturing the image stream
		status = scStartStream(device_);
		if (status != ScStatus::SC_OK)
		{
			cout << "scStartStream "<<pDeviceListInfo->serialNumber<<" failed status:" << status << endl;
			isTestDone_ = true;
			condition_.notify_one();
			return;
		}

        //Wait for the device to upload image data
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// 1.ReadNextFrame.
		// 2.Get depth Frame acoording to Ready flag.
		for (int j = 0; j < frameSpace; j++)
		{
			ScFrame depthFrame = {0};
			ScFrameReady frameReady = {0};
			status = scGetFrameReady(device_, 1200, &frameReady);

			if (status != SC_OK)
			{
				cout << pDeviceListInfo->serialNumber << " scGetFrameReady failed status:" << status << endl;
				continue;
			}

			// Get depth frame, depth frame only output in following data mode
			if (1 == frameReady.depth)
			{
				status = scGetFrame(device_, SC_DEPTH_FRAME, &depthFrame);

				if (status == SC_OK && depthFrame.pFrameData != NULL)
				{
					cout << pDeviceListInfo->serialNumber << " frameIndex :" << depthFrame.frameIndex << endl;
				}
				else
				{
					cout << pDeviceListInfo->serialNumber << "scGetFrame SC_DEPTH_FRAME status:" << status << endl;
				}
			}
		}

		// 1.close device
		// 2.SDK shutdown
		status = scStopStream(device_);
		if (status != ScStatus::SC_OK)
		{
			cout << "scStopStream failed status:" << status << endl;
		}
		status = scCloseDevice(&device_);
		if (status != ScStatus::SC_OK)
		{
			cout << "scCloseDevice failed status:" << status << endl;
		}

        isTestDone_ = true;
        condition_.notify_one();

		return;
	}

    bool WaitForTestDone()
    {
        unique_lock<mutex> lk(mutex_);
        condition_.wait(lk, [this] {return isTestDone_;});
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
	cout << "---MultiConnectionInMultiThread---"<< endl;

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
		cout << "scInitialize failed status:" <<status << endl;
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
			cout << "scGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
			system("pause");
			return -1;
		}
		cout << "Get device count: " << deviceCount << endl;
	} while (deviceCount < 2);

	deviceHandle = new ScDeviceHandle[deviceCount];
	pDeviceListInfo = new ScDeviceInfo[deviceCount];


	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status != ScStatus::SC_OK)
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		delete[] pDeviceListInfo;
		delete[] deviceHandle;
		return -1;
	}
	else
	{
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection." << endl;
			delete[] pDeviceListInfo;
			delete[] deviceHandle;
			return -1;
		}
		Device* pDevice = new Device[deviceCount];

		for(uint32_t i = 0; i < deviceCount; i++)
		{
            thread(&Device::TestDevice, std::ref(pDevice[i]), &pDeviceListInfo[i]).detach();
		}

		for(uint32_t i = 0; i < deviceCount; i++)
		{
            pDevice[i].WaitForTestDone();
		}

		delete[] pDevice;
		delete[] pDeviceListInfo;
		delete[] deviceHandle;
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
