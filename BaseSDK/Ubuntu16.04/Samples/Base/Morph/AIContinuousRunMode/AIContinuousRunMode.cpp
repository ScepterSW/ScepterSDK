#include <thread>
#include <iostream>
#include <iomanip>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"

#define ResultCount 100
using namespace std;

ScStatus SetParams(ScDeviceHandle deviceHandle)
{
	ScStatus status = SC_OTHERS;
	status = scSetFrameRate(deviceHandle, 15);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetFrameRate] success, ScStatus(" << status << "). Set the device frame rate to 15." << endl;
	}
	else
	{
		cout << "[scSetFrameRate] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scSetColorResolution(deviceHandle, 640, 480);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scSetColorResolution] success, ScStatus(" << status << "). Set color sensor resolution 640 * 480." << endl;
	}
	else
	{
		cout << "[scSetColorResolution] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	//Get the parameter with the parameter ID 4.
	uint32_t paramID = 4;
	void* pBuffer = 0;
	uint16_t nBufferSize = 0;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}
	string strBuffer = (char*)pBuffer;
	cout << "[scAIModuleGetParam] paramID: 4 status:" << status << "   text(ASCII): " << strBuffer.c_str() << endl;

	//Set the parameter with the parameter ID 4.
	char buffer_ASCII[13] = { "Hello world." };
	status = scAIModuleSetParam(deviceHandle, paramID, buffer_ASCII, sizeof(buffer_ASCII));
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleSetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}
	cout << "[scAIModuleSetParam] paramID: 4 status:" << status << "   text(ASCII): " << buffer_ASCII << endl;

	//Get the parameter with the parameter ID 4.
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}
	strBuffer = (char*)pBuffer;
	cout << "[scAIModuleGetParam] paramID: 4 status:" << status << "   text(ASCII): " << strBuffer.c_str() << endl;

	//Get the parameter with the parameter ID 5.
	paramID = 5;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	uint8_t* pbuf = (uint8_t*)pBuffer;
	cout << "[scAIModuleGetParam] paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < nBufferSize; i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(*(pbuf + i)) << " ";
	}
	cout << dec << endl;

	//Set the parameter with the parameter ID 5.
	uint8_t buffer_HEX[3] = { 0x01, 0x02, 0x03 };
	status = scAIModuleSetParam(deviceHandle, paramID, buffer_HEX, sizeof(buffer_HEX));
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleSetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}
	cout << "[scAIModuleSetParam] paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < sizeof(buffer_HEX); i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(buffer_HEX[i]) << " ";
	}
	cout << dec << endl;

	//Get the parameter with the parameter ID 5.
	paramID = 5;
	status = scAIModuleGetParam(deviceHandle, paramID, &pBuffer, &nBufferSize);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleGetParam] success, ScStatus(" << status << "). ";
	}
	else
	{
		cout << "[scAIModuleGetParam] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	pbuf = (uint8_t*)pBuffer;
	cout << "[scAIModuleGetParam] paramID: 5 status:" << status << "   text(HEX): " << hex;
	for (int i = 0; i < nBufferSize; i++)
	{
		cout << "0x" << setfill('0') << std::setw(2) << static_cast<int>(*(pbuf + i)) << " ";
	}
	cout << dec << endl;

	status = scAIModuleSetInputFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, true);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetInputFrameTypeEnabled] success, ScStatus(" << status << "). Set SC_COLOR_FRAME true." << endl;
	}
	else
	{
		cout << "[scAIModuleSetInputFrameTypeEnabled] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scAIModuleSetInputFrameTypeEnabled(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, true);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetInputFrameTypeEnabled] success, ScStatus(" << status << "). Set SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME true." << endl;
	}
	else
	{
		cout << "[scAIModuleSetInputFrameTypeEnabled] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_DEPTH_FRAME, false);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetPreviewFrameTypeEnabled] success, ScStatus(" << status << "). Set SC_DEPTH_FRAME false." << endl;
	}
	else
	{
		cout << "[scAIModuleSetPreviewFrameTypeEnabled] fail, ScStatus(" << status << ")." << endl;
		return status;
	}

	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_IR_FRAME, false);
	if (status == ScStatus::SC_OK)
	{
		cout << "[scAIModuleSetPreviewFrameTypeEnabled] success, ScStatus(" << status << "). Set SC_IR_FRAME false." << endl;
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
	cout << "---AIContinuousRunMode---" << endl;

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
			return -1;
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
		return -1;
	}

	for (int j = 0; j < ResultCount; j++)
	{
		ScAIResult aiResult = { 0 };
		ScStatus status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		if (status == ScStatus::SC_OK)
		{
			cout << "[scAIModuleGetResult] success, ScStatus(" << status << "). ScAIResult <resultIndex>:" << aiResult.resultIndex << endl;
		}
		else
		{
			cout << "[scAIModuleGetResult] fail, ScStatus(" << status << ")." << endl;
			continue;
		}
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
	cout << "---end---" << endl;

	return 0;
}
