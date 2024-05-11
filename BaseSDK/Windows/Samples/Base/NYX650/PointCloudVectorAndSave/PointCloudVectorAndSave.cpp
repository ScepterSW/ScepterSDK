#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>
#define frameSpace 20

using namespace std;

int main()
{
	cout << "---PointCloudVectorAndSave---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	
	//about frame
	
	ScFrameReady FrameReady = { 0 };
	ScFrame depthFrame = { 0 };
	ofstream PointCloudWriter;

	//SDK Initialize
	status = scInitialize();
	if (status != ScStatus::SC_OK)
	{
		cout << "ScInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
	status = scGetDeviceCount(&deviceCount, 3000);
	if (status != ScStatus::SC_OK)
	{
		cout << "ScGetDeviceCount failed! make sure pointer valid or called scInitialize()" << endl;
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

	pDeviceListInfo = new ScDeviceInfo[deviceCount];
	status = scGetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status == ScStatus::SC_OK)
	{
		if (SC_CONNECTABLE != pDeviceListInfo[0].status)
		{
			cout << "connect status: " << pDeviceListInfo[0].status << endl;
			cout << "The device state does not support connection."<< endl;
			return -1;
		}
	}
	else
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		return -1;
	}

	cout << "serialNumber:" << pDeviceListInfo[0].serialNumber << endl
		<< "ip:" << pDeviceListInfo[0].ip << endl
		<< "connectStatus:" << pDeviceListInfo[0].status << endl;

	status = scOpenDeviceBySN(pDeviceListInfo[0].serialNumber, &deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return -1;
	}

    cout << "open device successful,status :" << status << endl;

	ScSensorIntrinsicParameters cameraParam = {};
	scGetSensorIntrinsicParameters(deviceHandle, SC_TOF_SENSOR, &cameraParam);

	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" <<status<< endl;
		return -1;
	}

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//1.ReadNextFrame.
	//2.GetFrame acoording to Ready flag and Frametype.
	//3.save points.
	for (int i = 0; i < frameSpace; i++)
	{
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" <<status<< endl;
			continue;
		}

		//depthFrame only.
		if (1 == FrameReady.depth)
		{
			status = scGetFrame(deviceHandle, SC_DEPTH_FRAME, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				// once save
				
				PointCloudWriter.open("PointCloud.txt");
				ScFrame &srcFrame = depthFrame;
				const int WINDOW_SIZE = 100;

				const uint16_t* pDepthFrameData = (uint16_t*)srcFrame.pFrameData;
				for (int i = (srcFrame.height - WINDOW_SIZE)/2, offset = i * srcFrame.width; i < (srcFrame.height + WINDOW_SIZE)/2; i++)
				{
					for (int j = (srcFrame.width - WINDOW_SIZE)/2; j < (srcFrame.width + WINDOW_SIZE)/2; j++)
					{
						ScDepthVector3 depthPoint = {j, i, pDepthFrameData[offset + j]};
						ScVector3f worldV = {};
						scConvertDepthToPointCloud(deviceHandle, &depthPoint, &worldV, 1, &cameraParam);
						if (0 < worldV.z && worldV.z < 0xFFFF)
						{
							PointCloudWriter << worldV.x << "\t" << worldV.y << "\t" << worldV.z << std::endl;
						}
					}
					offset += srcFrame.width;
				}
				std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
				PointCloudWriter.close();
				break;
			}
		}

	}


	//StoSc capturing the image stream
	status = scStopStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStopStream failed status:" <<status<< endl;
		return -1;
	}

	//1.close device
	//2.SDK shutdown
	status = scCloseDevice(&deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scCloseDevice failed status:" <<status<< endl;
		return -1;
	}
	status = scShutdown();
	if (status != ScStatus::SC_OK)
	{
		cout << "scShutdown failed status:" <<status<< endl;
		return -1;
	}
	cout << "---end---"<< endl;

	return 0;
}
