#include <iostream>
#ifdef _WIN32
#include <sys/timeb.h>
#else
#include <time.h>
#endif
#include <thread>
#include <fstream>
#include <string>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"
#include "get_cpu_vmem.h"
using namespace std;

uint64_t getCurrentTime()
{
#ifdef _WIN32
    timeb timeInfo;
    ftime(&timeInfo);
    return (timeInfo.time * 1000 + timeInfo.millitm);
#else
    timespec timeInfo;
    clock_gettime(CLOCK_REALTIME, &timeInfo);
    return (timeInfo.tv_sec * 1000 + timeInfo.tv_nsec/1000000);
#endif
} 

ScStatus SetParams(ScDeviceHandle deviceHandle)
{
	ScStatus status = SC_OTHERS;

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
	// Must use GUI tool import specific configuration first
	cout << "---AISingleTriggerResultDelayTest---" << endl;
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

	//Initialize the ScepterSDK.
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

	//Get the count of devices.
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

	//Get the infomation of devices.
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

	// set soft work model
	status = scSetWorkMode(deviceHandle, SC_SOFTWARE_TRIGGER_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return 1;
	}
	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
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

	//Start the data stream.
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

	ofstream csvWriter;
	string fileName = "";
	csvWriter.open(fileName.append("AISingleTriggerResultDelayTest.csv"));
	if (!csvWriter.is_open())
	{
		cout << "csv file open failed" << endl;
		return 1;
	}
	csvWriter << "resultIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure" << endl;

	uint64_t endTimestamp = 0;
	uint64_t startTimestamp = 0;
	uint64_t deviceTimestamp = 0;
	uint64_t frameInterval = 0;
	uint64_t frameIntervalNTP = 0;
	int current_pid = GetCurrentPid();
	float cpu = 0;
	float vmem = 0;
	ScAIResult aiResult = { 0 };
	uint32_t number = 100;
	for (uint32_t i = 0; i < number; i++)
	{
		status = scSoftwareTriggerOnce(deviceHandle);
		startTimestamp = getCurrentTime();
		if (status != ScStatus::SC_OK)
		{
			cout << "[scSoftwareTriggerOnce] fail, ScStatus(" << status << ")." << endl;
			continue;
		} 

		status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		endTimestamp = getCurrentTime();
		if (status != SC_OK)
		{
			cout << "[scAIModuleGetResult] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		if (aiResult.resultIndex % 10 == 0)
		{
			cout << "[scAIModuleGetResult],ScStatus(" << status << "). "
				<< "resultIndex:" << aiResult.resultIndex << "  "
				<< "resultTimestamp:" << aiResult.resultTimestamp << endl;
		}

		deviceTimestamp = aiResult.resultTimestamp;
		frameInterval = endTimestamp - startTimestamp;
		frameIntervalNTP = endTimestamp - deviceTimestamp;
		cpu = GetCpuUsageRatio(current_pid);
		vmem = GetMemoryUsage(current_pid);
		if (frameIntervalNTP > 2000)
		{
			csvWriter << aiResult.resultIndex << "," << cpu << "," << vmem << "," << frameInterval <<  ",N/A" << endl;
		}
		else
		{
			csvWriter << aiResult.resultIndex << "," << cpu << "," << vmem << "," << frameInterval << "," << frameIntervalNTP << endl;
		}
		this_thread::sleep_for(chrono::milliseconds(1000 / frameRate));
	}
	csvWriter.close();
	status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetWorkMode] success, ScStatus(" << status << "). Set SC_ACTIVE_MODE." << endl;
	}
	else
	{
		cout << "[scSetWorkMode] fail, ScStatus(" << status << ")." << endl;
		return 1;
	}
	//Close the opened device.
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
