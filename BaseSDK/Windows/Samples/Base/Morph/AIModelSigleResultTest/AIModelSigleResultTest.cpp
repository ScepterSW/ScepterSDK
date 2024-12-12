#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"
#include "get_cpu_vmem.h"
#ifdef _WIN32
#include <sys/timeb.h>
#else
#include <time.h>
#endif

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
	//Set frame rate.
	status = scSetFrameRate(deviceHandle, 30);
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

	//Set preview frame type of SC_COLOR_FRAME disabled.
	status = scAIModuleSetPreviewFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, false);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleSetPreviewFrameTypeEnabled IR failed status:" << status << endl;
		return status;
	}

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

	//Set AI module single running.
	status = scAIModuleSetWorkMode(deviceHandle, AI_SINGLE_RUN_MODE);
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

int displayKeyconfiguration(const ScDeviceHandle deviceHandle)
{
	// key configuration display
	ScStatus status = SC_OTHERS;
	char version[64] = { 0 };
	status = scGetSDKVersion(version, 64);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetSDKVersion failed status:" << status << endl;
		return -1;
	}
	cout << "********scGetSDKVersion: " << version <<"********"<< endl;
	status = scGetFirmwareVersion(deviceHandle, version, 64);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFirmwareVersion failed status:" << status << endl;
		return -1;
	}
	cout << "********scGetFirmwareVersion: " << version <<"********"<< endl;
	bool enable = false;
	status = scAIModuleGetEnabled(deviceHandle, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetEnabled: " << enable <<"********"<< endl;
	if (enable == false)
	{
		cout << "******** Config File ERROR, Please check the config file first ********"<< endl;
		// status = scAIModuleSetEnabled(deviceHandle, true);
		// if (status != ScStatus::SC_OK)
		// {
		// 	cout << "scAIModuleGetEnabled failed status:" << status << endl;
		// 	return -1;
		// }
	}
	ScWorkMode pMode;
	status = scGetWorkMode(deviceHandle, &pMode);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetWorkMode failed status:" << status << endl;
		return -1;
	}
	cout << "********scGetWorkMode : " << pMode <<"********"<< endl;
	ScAIModuleMode mode;
	status = scAIModuleGetWorkMode(deviceHandle, &mode);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetWorkMode failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetWorkMode : " << mode <<"********"<< endl;
	status = scAIModuleGetInputFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetInputFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetInputFrameTypeEnabled SC_COLOR_FRAME: " << enable <<"********"<< endl;
	status = scAIModuleGetInputFrameTypeEnabled(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetInputFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME :" << enable <<"********"<< endl;

	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_COLOR_FRAME: " << enable <<"********"<< endl;
	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_DEPTH_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_DEPTH_FRAME: " << enable <<"********"<< endl;

	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_IR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_IR_FRAME: " << enable <<"********"<< endl;
	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "********scGetFrameRate : " << frameRate << "********" << endl;
	}
	int32_t pW;
	int32_t pH;
	status = scGetColorResolution(deviceHandle, &pW, &pH);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetColorResolution failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "********scGetColorResolution pw: " << pW << " ph: " << pH << "********" << endl;
	}
}


int main()
{
	cout << "---AIModelSigleResultTest---" << endl;

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

	// config param through GUI tool
	//if (false)
	//{
	//	status = SetParams(deviceHandle);
	//	if (status != ScStatus::SC_OK)
	//	{
	//		cout << "SetParams failed status:" << status << endl;
	//		return -1;
	//	}
	//}

	displayKeyconfiguration(deviceHandle);
	//Start the data stream.
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}
	//Wait for the device to upload image data
	ofstream csvWriter;
	string fileName = "";
	csvWriter.open(fileName.append("AIModelSigleResultTest.csv"));
	if (!csvWriter.is_open())
	{
		cout << "csv file open failed" << endl;
		return -1;
	}
	csvWriter << "resultIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure" << endl;
	//1.software trigger.
	//2.ReadNextFrame.
	//3.GetFrame acoording to Ready flag and Frametype.
	//4.sleep 1000/frameRate (ms)
	uint64_t endTimestamp = 0;
	uint64_t startTimestamp = 0;
	uint64_t deviceTimestamp = 0;
	uint64_t frameInterval = 0;
	uint64_t frameIntervalNTP = 0;
	int current_pid = GetCurrentPid();
	float cpu = 0;
	float vmem = 0;
	ScAIResult aiResult = { 0 };
	cout << "Please input the number of tests:" << endl;
	uint32_t number = 0;
	cin >> number;
	//Get the result form AI module.
	for (int i = 0; i < number; i++)
	{
		//Call the below api to trigger one result, then the result will be sent.
		// If do not call this function, the result will not be sent and the API of scAIModuleGetResult will return timeout fail.
		status = scAIModuleTriggerOnce(deviceHandle);
		startTimestamp = getCurrentTime();
		if (status != ScStatus::SC_OK)
		{
			cout << "scAIModuleTriggerOnce failed status:" << status << endl;
			continue;
		}

GOTOTAG:
		ScStatus status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		endTimestamp = getCurrentTime();
		if (status != SC_OK)
		{
			cout << " scAIModuleGetResult failed status:" << status << endl;
			continue;
		}
		string tmp((char*)aiResult.pResultData);
		if (tmp.find('{') == -1)
		{
			cout << "The last usless res: " << aiResult.resultIndex << endl;
			goto GOTOTAG;
		}
		if (aiResult.resultIndex % 10 == 0)
		{
			cout << "AIModuleGetResult resultIndex :" << aiResult.resultIndex  << endl;
		}
		deviceTimestamp = aiResult.resultTimestamp; // actually it equals to the depthFrame.deviceTimestamp
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
	cout << "---end---" << endl;

	return 0;
}
