#include <thread>
#include <iostream>
#include <iomanip>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"

#define ResultCount 10
using namespace std;

ScStatus SetParams(ScDeviceHandle deviceHandle)
{
	ScStatus status = SC_OTHERS;
	
	status = scAIModuleSetWorkMode(deviceHandle, AI_SINGLE_RUN_MODE);
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
	cout << "---AISingleRunMode---" << endl;

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
	int frameRate = 5;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scGetFrameRate] success, ScStatus(" << status << "). The device frame rate is " << frameRate << endl;
	}
	else
	{
		cout << "[scGetFrameRate] fail, ScStatus(" << status << ")." << endl;
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

	for (int j = 0; j < ResultCount; j++)
	{
		//Call the below api to trigger one result, then the result will be sent.
		// If do not call this function, the result will not be sent and the API of scAIModuleGetResult will return timeout fail.
		status = scAIModuleTriggerOnce(deviceHandle);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scAIModuleTriggerOnce] success, ScStatus(" << status << ")." << endl;
		}
		else
		{
			cout << "[scAIModuleTriggerOnce] fail, ScStatus(" << status << ")." << endl;
			continue;
		}

		ScAIResult aiResult = { 0 };
		status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scAIModuleGetResult] success, ScStatus(" << status << "). ScAIResult <resultIndex>:" << aiResult.resultIndex << endl;
		}
		else
		{
			cout << "[scAIModuleGetResult] fail, ScStatus(" << status << ")." << endl;
			continue;
		}
		this_thread::sleep_for(chrono::milliseconds(1200 / frameRate));
	}

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
