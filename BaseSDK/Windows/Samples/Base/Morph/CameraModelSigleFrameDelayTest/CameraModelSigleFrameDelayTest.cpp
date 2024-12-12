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
	if (enable)
	{
		cout << "******** Config File ERROR, Please check the config file first ********"<< endl;
		// cout << "******** disable AI ********"<< endl;
		// status = scAIModuleSetEnabled(deviceHandle, false);
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
	cout << "********scGetWorkMode : " << pMode <<" ********"<< endl;
	ScAIModuleMode mode;
	status = scAIModuleGetWorkMode(deviceHandle, &mode);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetWorkMode failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetWorkMode : " << mode <<" ********"<< endl;

	status = scAIModuleGetInputFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetInputFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetInputFrameTypeEnabled SC_COLOR_FRAME: " << enable <<" ********"<< endl;
	status = scAIModuleGetInputFrameTypeEnabled(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetInputFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME :" << enable <<" ********"<< endl;

	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_COLOR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_COLOR_FRAME: " << enable <<" ********"<< endl;
	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_DEPTH_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_DEPTH_FRAME: " << enable <<" ********"<< endl;

	status = scAIModuleGetPreviewFrameTypeEnabled(deviceHandle, SC_IR_FRAME, &enable);
	if (status != ScStatus::SC_OK)
	{
		cout << "scAIModuleGetPreviewFrameTypeEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "********scAIModuleGetPreviewFrameTypeEnabled SC_IR_FRAME: " << enable <<" ********"<< endl;
	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "********scGetFrameRate : " << frameRate << " ********" << endl;
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
		cout << "********scGetColorResolution pw: " << pW << " ph: " << pH << " ********" << endl;
	}
	ScTimeFilterParams pParams;
	scGetTimeFilterParams(deviceHandle, &pParams);
	cout << "********scGetTimeFilterParams: " << pParams.enable << " ********"<< endl;
}
int main()
{
	// Must use GUI tool import specific configuration first
	cout << "---CameraModelSigleFrameDelayTest---" << endl;
	ScStatus status = SC_OTHERS;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "scInitialize failed status:" << status << endl;
		system("pause");
		return -1;
	}

	uint32_t deviceCount = 0;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
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

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection." << endl;
			return -1;
		}
	}
	else
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		return -1;
	}

	cout << "productName: " << pDeviceListInfo[0].productName << endl
	 	<< "serialNumber:" << pDeviceListInfo[0].serialNumber << endl
		<< "ip:" << pDeviceListInfo[0].ip << endl
		<< "connectStatus:" << pDeviceListInfo[0].status << endl;

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" << status << endl;
		return false;
	}
	// set soft work model
	status = scSetWorkMode(deviceHandle, SC_SOFTWARE_TRIGGER_MODE);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetWorkMode failed status:" << status << endl;
		return -1;
	}
	int frameRate = 0;
	status = scGetFrameRate(deviceHandle, &frameRate);
	if (status != ScStatus::SC_OK)
	{
		cout << "scGetFrameRate failed status:" << status << endl;
		return -1;
	}
	displayKeyconfiguration(deviceHandle);
	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}
	//Wait for the device to upload image data
	ScFrameReady FrameReady = { 0 };
	ScFrame depthFrame = { 0 };
	ofstream csvWriter;
	string fileName = "";
	csvWriter.open(fileName.append("CameraModelSigleFrameDelayTest.csv"));
	if (!csvWriter.is_open())
	{
		cout << "csv file open failed" << endl;
		return -1;
	}
	csvWriter << "frameIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure" << endl;
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
	cout << "Please input the number of tests:" << endl;
	uint32_t number = 0;
	cin >> number;
	for (int i = 0; i < number; i++)
	{
		//call the below api to trigger one frame, then the frame will be sent
		// if do not call this function, the frame will not be sent and the below call will return timeout fail
		status = scSoftwareTriggerOnce(deviceHandle);
		// ftime(&timeStart); // record the start timestamp
		startTimestamp = getCurrentTime();
		if (status != ScStatus::SC_OK)
		{
			cout << "scSoftwareTriggerOnce failed status:" << status << endl;
			continue;
		}
		//If no image is ready within 1200ms, the function will return ScRetGetFrameReadyTimeOut
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" << status << endl;
			continue;
		}
		if (1 == FrameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				if (depthFrame.frameIndex % 10 == 0)
				{
					cout << "scGetFrame,status:" << status << "  "
						<< "frameType:" << depthFrame.frameType << "  "
						<< "frameIndex:" << depthFrame.frameIndex << endl;
				}
				//ftime(&timeEnd);// record the end timestamp
				endTimestamp = getCurrentTime();
				deviceTimestamp = depthFrame.deviceTimestamp;
				frameInterval = endTimestamp - startTimestamp;
				frameIntervalNTP = endTimestamp - deviceTimestamp;
				cpu = GetCpuUsageRatio(current_pid);
				vmem = GetMemoryUsage(current_pid);
				if (frameIntervalNTP > 2000)
				{
					csvWriter << depthFrame.frameIndex << "," << cpu << "," << vmem << "," << frameInterval <<  ",N/A" << endl;
				}
				else
				{
					csvWriter << depthFrame.frameIndex << "," << cpu << "," << vmem << "," << frameInterval << "," << frameIntervalNTP << endl;
				}
			}
		}
		this_thread::sleep_for(chrono::milliseconds(1000 / frameRate));
	}
	csvWriter.close();

	// //set slave false
	// status = scSetWorkMode(deviceHandle, SC_ACTIVE_MODE);
	// if (status != ScStatus::SC_OK)
	// {
	// 	cout << "scSetWorkMode failed status:" << status << endl;
	// 	return -1;
	// }

	//Stop capturing the image stream
	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" << status << endl;
		return -1;
	}

	//1.close device
	//2.SDK shutdown
	status = scCloseDevice(&deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scCloseDevice failed status:" << status << endl;
		return -1;
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" << status << endl;
		return -1;
	}
	cout << "---end---" << endl;

	return 0;
}
