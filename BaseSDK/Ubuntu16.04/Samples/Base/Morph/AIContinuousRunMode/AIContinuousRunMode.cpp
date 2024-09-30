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
	cout << "scAIModuleGetParam paramID: 4 status:" << status <<"   text(ASCII): " << strBuffer.c_str() <<endl;

	//Set the parameter with the parameter ID 4.
	char buffer_ASCII[13] = {"Hello world."};
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
	for (int i = 0; i < nBufferSize;i++)
	{
		cout << "0x" <<setfill('0') << std::setw(2) << static_cast<int>(*(pbuf + i)) << " ";
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
		cout << "0x" << setfill('0') << std::setw(2) << hex << static_cast<int>(buffer_HEX[i]) << " ";
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
	cout << "---AIContinuousRunMode---"<< endl;

	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;

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
		cout << "scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples."<< endl;
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
			cout << "The device state does not support connection."<< endl;
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

	//Get the result form AI module.
	for (int j = 0; j < ResultCount; j++)
	{
		ScAIResult aiResult = { 0 };
		ScStatus status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		if (status != SC_OK)
		{
			cout << " scAIModuleGetResult failed status:" << status << endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		cout << "AIModuleGetResult resultIndex :" << aiResult.resultIndex << endl;
	}

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
	cout << "---end---"<< endl;

	return 0;
}
