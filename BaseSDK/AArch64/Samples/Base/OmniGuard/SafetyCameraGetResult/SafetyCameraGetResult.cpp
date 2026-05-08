#include <thread>
#include <iostream>
#include <iomanip>
#include "Scepter_api.h"
#include "Scepter_Morph_api.h"
#include <vector>

#define ResultCount 100
using namespace std;
#pragma pack(push, 1)
typedef struct
{
	uint8_t index;
	uint8_t safetyLevel;
	uint8_t isWarning;
} DetectedResult;

struct AlgHeader
{
	/*
	//Start identifer:STX A L G
	pBuf[0] = 0x02; //STX
	pBuf[1] = 0x41; //A
	pBuf[2] = 0x4C; //L
	pBuf[3] = 0x47; //G
	uint32_t startIdentifer = 0x474C4102;
	*/
	uint32_t startIdentifer;
	uint16_t sizeOfValidData;
	uint8_t  checksum;
};
#pragma pack(pop)

int ParseResult(ScAIResult result, vector<DetectedResult>& detectedResult)
{
	uint32_t usedLen = 0;

	if (result.dataLen < sizeof(AlgHeader))
	{
		cout << "dataLen is invalid. " << result.dataLen << endl;
		return 1;
	}
	// header
	const AlgHeader& algHeader = *(const AlgHeader*)(result.pResultData);
	if (0x474C4102 != algHeader.startIdentifer)
	{
		cout << "startIdentifer is invalid. " << algHeader.startIdentifer << endl;
		return 1;
	}
	usedLen += sizeof(AlgHeader);

	if (result.dataLen < (usedLen + algHeader.sizeOfValidData))
	{
		cout << "sizeOfValidData is invalid. " << algHeader.sizeOfValidData << endl;
		return 1;
	}

	// check checksum
	uint8_t checksum = 0;
	for (int i = 0; i < algHeader.sizeOfValidData; i++)
	{
		checksum += *(const uint8_t*)(result.pResultData + usedLen + i);
	}

	if (checksum != algHeader.checksum)
	{
		cout << "checkSum is invalid. " << (int)checksum << " algHeader.checksum " << (int)algHeader.checksum << endl;
		return 1;
	}

	const uint8_t* pPackageID = (const uint8_t*)(result.pResultData + usedLen);
	usedLen++;

	if (0x00 != pPackageID[0])
	{
		cout << "pPackageID is invalid " << (unsigned int)pPackageID[0] << endl;
		return 1;
	}

	uint16_t& detectedResultLen = *(uint16_t*)(result.pResultData + usedLen);
	usedLen += sizeof(uint16_t);
	uint8_t& state = *(uint8_t*)(result.pResultData + usedLen);
	usedLen += sizeof(uint8_t);

	uint8_t& errorCode = *(uint8_t*)(result.pResultData + usedLen);
	usedLen += sizeof(uint8_t);

	if (0 != state || 0 != errorCode)
	{
		cout << " Check failed state " << (unsigned int)state << " errorCode " << (unsigned int)state << " detectedResultLen " << detectedResultLen <<endl;
		return 1;
	}

	uint8_t& detectionAreaCount = *(uint8_t*)(result.pResultData + usedLen);
	usedLen += sizeof(uint8_t);

	for (size_t i = 0; i < detectionAreaCount; i++)
	{
		detectedResult.push_back(*(DetectedResult*)(result.pResultData + usedLen));
		usedLen += sizeof(DetectedResult);
	}

	if (result.dataLen > usedLen)
	{
		cout << " ignoring useless debug data " << result.dataLen << "  " << usedLen << "  " << (result.dataLen - usedLen) << endl;
	}

	return 0;
}

int main()
{
	cout << "---SafetyCameraGetResult---" << endl;

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
		ScAIResult aiResult = { 0 };
		status = scAIModuleGetResult(deviceHandle, 1200, &aiResult);
		if (status == ScStatus::SC_OK)
		{
			vector<DetectedResult> detectedResult;
			ParseResult(aiResult, detectedResult);

			for (int i = 0; i < (int)detectedResult.size(); i++)
			{
				if ((uint16_t)detectedResult[i].isWarning == 1)
				{
					cout << "area " << (uint16_t)detectedResult[i].index << " is dangerous." << endl;
				}
			}
		}
		else
		{
			cout << "[scAIModuleGetResult] fail, ScStatus(" << status << ")." << endl;
			continue;
		}
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
