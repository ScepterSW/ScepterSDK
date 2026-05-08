#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <fstream>
#include <sys/timeb.h>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"

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
				ScFrame depthFrame = { 0 };
				ScFrameReady frameReady = { 0 };
				ScStatus status = scGetFrameReady(device_, 1000, &frameReady);
				if (status != SC_OK)
				{
					cout << "[scGetFrameReady] fail, ScStatus(" << status << ")." << endl;
					continue;
				}

				// Get depth frame.
				if (1 == frameReady.depth)
				{
					status = scGetFrame(device_, SC_DEPTH_FRAME, &depthFrame);
					if (status == SC_OK && depthFrame.pFrameData != NULL)
					{
						lock_guard<mutex> lkFrame(frameMutex);
						if (frameQueue.size() >= QueueMaxNum)
						{
							frameQueue.pop();
							frameQueue.push(depthFrame);
						}
						else
						{
							frameQueue.push(depthFrame);
						}
					}
					else
					{
						cout << "[scGetFrame] fail, ScStatus(" << status << ")." << endl;
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
					cout << "[scAIModuleGetResult] fail, ScStatus(" << status << ")." << endl;
					continue;
				}
				{
					lock_guard<mutex> lkResult(resultMutex);
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
	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_DEPTH_FRAME, true);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetPreviewFrameTypeEnabled] success, ScStatus(" << status << "). Preview SC_DEPTH_FRAME." << endl;
	}
	else
	{
		cout << "[scAIModuleSetPreviewFrameTypeEnabled] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scAIModuleSetWorkMode(deviceHandle, AI_CONTINUOUS_RUN_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetWorkMode] success, ScStatus(" << status << "). Set AI_CONTINUOUS_RUN_MODE." << endl;
	}
	else
	{
		cout << "[scAIModuleSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scAIModuleSetEnabled(deviceHandle, true);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetEnabled] success, ScStatus(" << status << "). Enable AI module." << endl;
	}
	else
	{
		cout << "[scAIModuleSetEnabled] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	return SC_OK;
}

int main()
{
	cout << "---AIGetFrameAndResultSync---" << endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	status = scInitialize();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scInitialize] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scInitialize] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	status = scGetDeviceCount(&deviceCount, 3000);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetDeviceCount] success, ScStatus(" << status << "). The device count is " << deviceCount << endl;
	}
	else
	{
		cout << "[scGetDeviceCount] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	if (0 == deviceCount)
	{
		cout << "[scGetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples." << endl;
		return 1;
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
			return 1;
		}
	}
	else
	{
		cout << "[scGetDeviceInfoList] fail, ScStatus(" << status << ")." << endl;
		delete[] pDeviceListInfo;
		pDeviceListInfo = NULL;
		return 1;
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
		return 1;
	}
	
	void*    pBuffer = 0;
	uint16_t nBufferSize = 0;
	// Get algo version
	uint32_t paramID = 0;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). The algo's version is: " << (char*)pBuffer << endl;
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	// Get algo name
	paramID = 1;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). The algo's name is: " << (char*)pBuffer << endl;
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	//Set bSetParams to false. if the params has been initialized by "Parameter initialization file" in ScepterGUITool or by SDK API in "DeviceImportParamInitFile" sample.
	bool bSetParams = true;
	if (bSetParams)
	{
		status = SetParams(deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[SetParams] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[SetParams] fail, ScStatus(" << status << ")." << endl;
			return 1;
		}
	}

	status = scStartStream(deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scStartStream] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scStartStream] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	DealThread* pDealThread = new DealThread[2];
	pDealThread[0].SetDeviceHandle(deviceHandle);
	pDealThread[1].SetDeviceHandle(deviceHandle);

	thread(&DealThread::Deal, &(pDealThread[0]), 1).detach();
	thread(&DealThread::Deal, &(pDealThread[1]), 2).detach();

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
					cout << "Sync result not match bigger, resultIndex: " << tAIResult.resultIndex << ", timestamp: " << tAIResult.resultTimestamp<< ", frameIndex:" << tFrame.frameIndex << ", timestamp: " << tFrame.deviceTimestamp << endl;
					resultQueue.pop();
				}
				else if (tFrame.deviceTimestamp < tAIResult.resultTimestamp)
				{
					cout << "Sync frame not match less, frameIndex:" << tFrame.frameIndex << ", timestamp: " << tFrame.deviceTimestamp << ", resultIndex: " << tAIResult.resultIndex << ", timestamp : " << tAIResult.resultTimestamp<< endl;
					frameQueue.pop();
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	pDealThread[0].SetRunning(false);
	pDealThread[1].SetRunning(false);
	for (uint32_t i = 0; i < 2; i++)
	{
		pDealThread[i].WaitForTestDone();
	}
	delete[] pDealThread;

	status = scCloseDevice(&deviceHandle);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scCloseDevice] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scCloseDevice] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}

	status = scShutdown();
	if (status == ScStatus::SC_OK)
	{
		cout << "[scShutdown] success, ScStatus(" << status << ")." << endl;
	}
	else
	{
		cout << "[scShutdown] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	cout << "---end---" << endl;

	return 0;
}
