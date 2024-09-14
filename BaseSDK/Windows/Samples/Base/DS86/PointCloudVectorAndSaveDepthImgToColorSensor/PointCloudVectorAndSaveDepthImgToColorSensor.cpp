#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>
#define frameSpace 20

using namespace std;

int main()
{
	cout << "---PointCloudVectorAndSaveDepthImgToColorSensor---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	
	//about frame
	
	ScFrameReady FrameReady = { 0 };
	ScFrame frame = { 0 };

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

    cout << "scOpenDeviceBySN status :" << status << endl;

	//switch ColorResolution
	int resolution_w = 800;
	int resolution_h = 600;
	status = scSetColorResolution(deviceHandle, resolution_w, resolution_h);
	if (status != ScStatus::SC_OK)
	{
		cout << "scSetColorResolution failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "set to 800_600" << endl;
	}
	//Starts capturing the image stream
	status = scStartStream(deviceHandle);
	if (status != ScStatus::SC_OK)
	{
		cout << "scStartStream failed status:" << status << endl;
		return -1;
	}
	//set Mapper
	status = scSetTransformDepthImgToColorSensorEnabled(deviceHandle, true);
	if (status == SC_OK)
	{
		cout << "scSetTransformDepthImgToColorSensorEnabled Enabled" << endl;
	}

	ScSensorIntrinsicParameters cameraParam = {};
	scGetSensorIntrinsicParameters(deviceHandle, SC_COLOR_SENSOR, &cameraParam);

	//1.ReadNextFrame.
	//2.GetFrame acoording to Ready flag and Frametype.
	//3.save points.
	for (int i = 0; i < frameSpace; i++)
	{
		//scGetFrameReady may return SC_GET_FRAME_READY_TIME_OUT.
		//Because it takes a little of time for scSetColorResolution to upload image data.
		status = scGetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != ScStatus::SC_OK)
		{
			cout << "scGetFrameReady failed status:" <<status<< endl;
			continue;
		}

		if (1 == FrameReady.transformedDepth)
		{
			status = scGetFrame(deviceHandle, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &frame);
			if (status == ScStatus::SC_OK&&frame.pFrameData != NULL)
			{
				cout << "scGetFrame status:" << status << "  "
					<< "frameType:" << frame.frameType << "  "
					<< "frameIndex:" << frame.frameIndex << endl;
			 
				// once save
				ofstream PointCloudWriter;
				
				PointCloudWriter.open("PointCloud.txt");
				ScFrame &srcFrame = frame;
				const int WINDOW_SIZE = 100;
				cout << srcFrame.width << "," << srcFrame.height << endl;
				const uint16_t* pDepthFrameData = (uint16_t*)srcFrame.pFrameData;
				for (int i = 0, offset = i * srcFrame.width; i < srcFrame.height; i++)
				{
					for (int j = 0; j < srcFrame.width ; j++)
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
