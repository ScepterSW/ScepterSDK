#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"

#include <fstream>
#include <sys/timeb.h>

#define ResultCount 100
#define QueueMaxNum	6
using namespace std;

mutex frameMutex;
mutex resultMutex;
std::queue<ScFrame> frameQueue;
std::queue<ScAIResult> resultQueue;

class DealThread
{
public:
	void Deal(int type)
	{

		lock_guard<mutex> lk(mutex_);
		// type: 1. get frame;2. get result of AI mode.
		if (1 == type)
		{
			while (running_)
			{
				ScFrame colorFrame = { 0 };
				ScFrameReady frameReady = { 0 };
				ScStatus status = scGetFrameReady(device_, 1000, &frameReady);
				if (status != SC_OK)
				{
					cout << " scGetFrameReady failed status:" << status << endl;
					continue;
				}

				// Get color frame.
				if (1 == frameReady.color)
				{
					status = scGetFrame(device_, SC_COLOR_FRAME, &colorFrame);
					if (status == SC_OK && colorFrame.pFrameData != NULL)
					{
						lock_guard<mutex> lk(frameMutex);
						if (frameQueue.size() >= QueueMaxNum)
						{
							frameQueue.pop();
							frameQueue.push(colorFrame);
						}
						else
						{
							frameQueue.push(colorFrame);
						}
					}
					else
					{
						cout << "scGetFrame SC_COLOR_FRAME status:" << status << endl;
					}
				}
			};
		}
		else
		{
			while (running_)
			{
				ScAIResult aiResult = { 0 };
				ScStatus status = scAIModuleGetResult(device_, 1000, &aiResult);
				if (status != SC_OK)
				{
					cout << " scAIModuleGetResult failed status:" << status << endl;
					continue;
				}
				{
					lock_guard<mutex> lk(resultMutex);
					if (resultQueue.size() >= QueueMaxNum)
					{
						resultQueue.pop();
						resultQueue.push(aiResult);
					}
					else
					{
						resultQueue.push(aiResult);
					}
				}
			};
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

	void SetDeviceHandle(ScDeviceHandle device)
	{
		device_ = device;
	}

	void SetRunning(bool running)
	{
		running_ = running;
	}

private:
	ScDeviceHandle device_;
	bool isTestDone_ = false;
	bool running_ = true;
	mutex mutex_;
	condition_variable condition_;
};

ScStatus SetParams(ScDeviceHandle deviceHandle)
{
	ScStatus status = SC_OTHERS;
	//Set frame rate.
	status = scSetFrameRate(deviceHandle, 15);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetFrameRate failed status:" << status << endl;
		return status;
	}
	//Set color resolution.
	status = scSetColorResolution(deviceHandle, 640, 480);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetColorResolution failed status:" << status << endl;
		return status;
	}

	//Get the parameter with the parameter ID 4.
	uint32_t paramID = 4;
	void* pBuffer = 0;
	uint16_t nBufferSize = 0;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetParam paramID: 4 failed status:" << status << endl;
		return status;
	}
	string strBuffer = (char*)pBuffer;
	cout << "scAIModuleGetParam paramID: 4 status:" << status << "   text(ASCII): " << strBuffer.c_str() << endl;

	//Set the parameter with the parameter ID 4.
	char buffer_ASCII[13] = { "Hello world." };
	status = scAIModuleSetParam(deviceHandle, paramID, buffer_ASCII, sizeof(buffer_ASCII));
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetParam paramID: 4 failed status:" << status << endl;
		return status;
	}
	cout << "scAIModuleSetParam paramID: 4 status:" << status << "   text(ASCII): " << buffer_ASCII << endl;

	//Get the parameter with the parameter ID 4.
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetParam paramID: 4 failed status:" << status << endl;
		return status;
	}
	strBuffer = (char*)pBuffer;
	cout << "scAIModuleGetParam paramID: 4 status:" << status << "   text(ASCII): " << strBuffer.c_str() << endl;

	//Get the parameter with the parameter ID 5.
	paramID = 5;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status != ScStatus::SC_OK || nBufferSize < 1)
	{
		cout << "scAIModuleGetParam paramID: 5 failed status:" << status << endl;
		return status;
	}

	uint8_t* pbuf = (uint8_t*)pBuffer;
	cout << "scAIModuleGetParam paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < nBufferSize; i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(*(pbuf + i)) << " ";
	}
	cout << dec << endl;

	//Set the parameter with the parameter ID 5.
	uint8_t buffer_HEX[3] = { 0x01, 0x02, 0x03 };
	status = scAIModuleSetParam(deviceHandle, paramID, buffer_HEX, sizeof(buffer_HEX));
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetParam paramID: 5 failed status:" << status << endl;
		return status;
	}
	cout << "scAIModuleSetParam paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < sizeof(buffer_HEX); i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(buffer_HEX[i]) << " ";
	}
	cout << dec << endl;

	//Get the parameter with the parameter ID 5.
	paramID = 5;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status != ScStatus::SC_OK || nBufferSize < 1)
	{
		cout << "scAIModuleGetParam paramID: 5 failed status:" << status << endl;
		return status;
	}

	pbuf = (uint8_t*)pBuffer;
	cout << "scAIModuleGetParam paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < nBufferSize; i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(*(pbuf + i)) << " ";
	}
	cout << dec << endl;

	//Set input frame type of SC_COLOR_FRAME enabled.
	status = scAIModuleSetInputFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, true);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetInputFrameTypeEnabled color failed status:" << status << endl;
		return status;
	}

	//Set input frame type of SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME enabled.
	status = scAIModuleSetInputFrameTypeEnabled(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, true);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetInputFrameTypeEnabled transforedDepth  failed status:" << status << endl;
		return status;
	}

	//Set preview frame type of SC_DEPTH_FRAME disabled.
	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_DEPTH_FRAME, false);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetPreviewFrameTypeEnabled depth failed status:" << status << endl;
		return status;
	}

	//Set preview frame type of SC_IR_FRAME disabled.
	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_IR_FRAME, false);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetPreviewFrameTypeEnabled IR failed status:" << status << endl;
		return status;
	}

	//Set AI module continuous running.
	status = scAIModuleSetWorkMode(deviceHandle, AI_CONTINUOUS_RUN_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetWorkMode failed status:" << status << endl;
		return status;
	}

	//Enable AI module.
	status = scAIModuleSetEnabled(deviceHandle, true);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetEnabled failed status:" << status << endl;
		return status;
	}

	return SC_OK;
}

int main()
{
	cout << "---AIGetFrameAndResultSync---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScStatus status = SC_OTHERS;
	ScDeviceHandle deviceHandle = 0;

	//Initialize the ScepterSDK.
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "scInitialize failed status:" << status << endl;
		system("pause");
		return -1;
	}

	//Get the count of devices.
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

	//Get the infomation of devices.
	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection." << endl; 
			delete[] pDeviceListInfo;
			return -1;
		}
	}
	else
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		delete[] pDeviceListInfo;
		return -1;
	}

	cout << "serialNumber:" << pDeviceListInfo[0].serialNumber << endl
		<< "ip:" << pDeviceListInfo[0].ip << endl
		<< "connectStatus:" << pDeviceListInfo[0].status << endl;
	//Open the first device by serial number.
	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" << status << endl;
		delete[] pDeviceListInfo;
		return false;
	}
	delete[] pDeviceListInfo;

	//Set bSetParams to false. if the params has been initialized by "Parameter initialization file" in ScepterGUITool or by SDK API in "xxxx" sample.
	bool bSetParams = true;
	if (bSetParams)
	{
		status = SetParams(deviceHandle);
		if (status != ScStatus::SC_OK)
		{
			cout << "SetParams failed status:" << status << endl;
			return -1;
		}
	}

	//Start the data stream.
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}

	//Create two threads to get the frame and the result of AI mode respectively.
	DealThread* pDealThread = new DealThread[2];
	pDealThread[0].SetDeviceHandle(deviceHandle);
	pDealThread[1].SetDeviceHandle(deviceHandle);

	//Get frame.
	thread(&DealThread::Deal, &(pDealThread[0]), 1).detach();
	//Get result of AI mode.
	thread(&DealThread::Deal, &(pDealThread[1]), 2).detach();

	//sync.
	ScFrame tFrame;
	ScAIResult tAIResult;
	int syncCount = ResultCount;
	while (syncCount > 0)
	{
		{
			lock_guard<mutex> lkResult(resultMutex);
			lock_guard<mutex> lkFrame(frameMutex);
			while (!resultQueue.empty() && !frameQueue.empty())
			{
				tAIResult = resultQueue.front();
				tFrame = frameQueue.front();

				if (tFrame.deviceTimestamp == tAIResult.resultTimestamp)
				{
					cout << "Sync frame index:" << tFrame.frameIndex << ", result index: " << tAIResult.resultIndex << ", timestamp: " << tAIResult.resultTimestamp << endl;
					frameQueue.pop();
					resultQueue.pop();
					syncCount--;
					break;
				}
				else if (tFrame.deviceTimestamp > tAIResult.resultTimestamp)
				{
					cout << "Sync result fail, resultIndex: " << tAIResult.resultIndex << ", timestamp: " << tAIResult.resultTimestamp<< ", frameIndex:" << tFrame.frameIndex << ", timestamp: " << tFrame.deviceTimestamp << endl;
					resultQueue.pop();
				}
				else if (tFrame.deviceTimestamp < tAIResult.resultTimestamp)
				{
					cout << "Sync frame fail, frameIndex:" << tFrame.frameIndex << ", timestamp: " << tFrame.deviceTimestamp << ", resultIndex: " << tAIResult.resultIndex << ", timestamp : " << tAIResult.resultTimestamp<< endl;
					frameQueue.pop();
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	//Stop running.
	pDealThread[0].SetRunning(false);
	pDealThread[1].SetRunning(false);
	//Wait for threads done.
	for (uint32_t i = 0; i < 2; i++)
	{
		pDealThread[i].WaitForTestDone();
	}
	delete[] pDealThread;

	//Stop the started stream.
	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" << status << endl;
		return -1;
	}

	//Close the opened device.
	status = scCloseDevice(&deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scCloseDevice failed status:" << status << endl;
		return -1;
	}

	//Shutdown the initialized ScepterSDK.
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" << status << endl;
		return -1;
	}
	cout << "---end---" << endl;

	return 0;
}
