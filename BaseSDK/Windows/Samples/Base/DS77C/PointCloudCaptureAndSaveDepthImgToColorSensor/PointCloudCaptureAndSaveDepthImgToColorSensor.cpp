#include <thread>
#include <iostream>
#include "Scepter_api.h"
#include <fstream>
#define frameSpace 20

using namespace std;

int main()
{
	cout << "---PointCloudCaptureAndSaveDepthImgToColorSensor---"<< endl;

	//about dev
	uint32_t deviceCount;
	ScDeviceInfo* pDeviceListInfo = NULL;
	ScDeviceHandle deviceHandle = 0;
	ScStatus status = SC_OTHERS;
	
	//about frame
	
	ScFrameReady FrameReady = {0};
	ScFrame frame = { 0 };

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
		return false;
	}

    cout << "open device successful,status :" << status << endl;

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

	//1.ReadNextFrame.
	//2.GetFrame acoording to Ready flag and Frametype.
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
				cout << "get Frame successful,status:" << status << "  "
					<< "frameTpye:" << frame.frameType << "  "
					<< "frameIndex:" << frame.frameIndex << endl;
				// once save
				ofstream PointCloudWriter;

				PointCloudWriter.open("PointCloud.txt");
				ScFrame &srcFrame = frame;
				const int len = srcFrame.width * srcFrame.height;
				ScVector3f* worldV = new ScVector3f[len];
				cout << srcFrame.width << "," << srcFrame.height << endl;

				scConvertDepthFrameToPointCloudVector(deviceHandle, &srcFrame, worldV); //Convert Depth frame to World vectors.

				for (int i = 0; i < len; i++)
				{
					if (0 < worldV[i].z && worldV[i].z < 0xFFFF)
					{
						PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
					}
				}
				delete[] worldV;
				worldV = NULL;
				std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
				PointCloudWriter.close();
				break;
			}
		}

	}
	//Stop capturing the image stream
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
