from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
print(libpath + "API")
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
from typing import List
import struct

class AlgHeader(Structure):
    _pack_ = 1
    _fields_ = [
        ("startIdentifer", c_uint32),
        ("sizeOfValidData", c_uint16),
        ("checksum", c_uint8)
    ]
class DetectedResult(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_uint8),
        ("safetyLevel", c_uint8),
        ("isWarning", c_uint8)
    ]
print("---SafetyCameraGetResult---")
camera = ScepterTofCam()
def parse_ai_result(result: ScAIResult):
    detected_results = []
    used_len = 0
    
    try:
        # 1. check data length
        header_size = ctypes.sizeof(AlgHeader)
        if result.dataLen < header_size:
            print(f"[ERROR] Insufficient data length: {result.dataLen} bytes (header requires {header_size} bytes)")
            return -1, []
        
        # 2. parse header
        header = AlgHeader.from_buffer_copy(
            ctypes.string_at(result.pResultData, header_size))
        
        if header.startIdentifer != 0x474C4102:
            print(f"[ERROR] Invalid header identifier: 0x{header.startIdentifer:08X} (expected 0x474C4102)")
            return -1, []
        
        used_len += header_size
        
        # 3. check vaild data length
        if result.dataLen < used_len + header.sizeOfValidData:
            print(f"[ERROR] Invalid data length: {header.sizeOfValidData} bytes specified but only {result.dataLen - used_len} bytes available")
            return -1, []
        
        # 4. vaild checksum
        data_slice = ctypes.string_at(
            ctypes.byref(result.pResultData.contents, used_len),
            header.sizeOfValidData)
        calculated_checksum = sum(data_slice) & 0xFF
        
        if calculated_checksum != header.checksum:
            print(f"[ERROR] Checksum mismatch: calculated 0x{calculated_checksum:02X}, expected 0x{header.checksum:02X}")
            return -1, []
        
        # 5. parse PackageID
        package_id = data_slice[0]
        used_len += 1
        if package_id != 0x00:
            print(f"[ERROR] Invalid package ID: 0x{package_id:02X} (expected 0x00)")
            return -1, []
        
        # 6. parse result length
        result_len = struct.unpack_from('<H', data_slice, 1)[0]
        used_len += 2
        
        # 7. parse state and error code
        state = data_slice[3]
        error_code = data_slice[4]
        used_len += 2
        
        if state != 0 or error_code != 0:
            print(f"[ERROR] State check failed - state: 0x{state:02X}, errorCode: 0x{error_code:02X}")
            return -1, []
        
        # 8. parse area count
        area_count = data_slice[5]
        used_len += 1
        
        # 9. parse detect result
        result_size = ctypes.sizeof(DetectedResult)
        offset = 6  # offset
        
        for _ in range(area_count):
            if offset + result_size > len(data_slice):
                print(f"[WARNING] Truncated data at result {len(detected_results) + 1}/{area_count}")
                break
            
            dr = DetectedResult.from_buffer_copy(data_slice, offset)
            detected_results.append(dr)
            offset += result_size
            used_len += result_size
        
        # 10. check the rest data
        if result.dataLen > used_len:
            remaining = result.dataLen - used_len
            print(f"[INFO] Result {result.resultIndex} has {remaining} bytes remaining data.")
        
        return 0, detected_results
    
    except Exception as e:
        print(f"[CRITICAL] Parsing failed: {str(e)}")
        return -1, []


camera_count = camera.scGetDeviceCount(3000)
print("[scGetDeviceCount] success, ScStatus(0), The device count is {}".format(camera_count))
if camera_count <= 0:
    print("[scGetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.")
    exit()

device_info=ScDeviceInfo()

if camera_count > 0:
    ret,device_infolist=camera.scGetDeviceInfoList(camera_count)
    if ret==0:
        device_info = device_infolist[0]
        print("[scGetDeviceInfoList] success, ScStatus({}). The first deviceInfo, <serialNumber>: {} , <ip>: {} , <status>: {}".format(ret, str(device_info.serialNumber.decode()), str(device_info.ip.decode()), str(device_info.status)))
        if  ScConnectStatus.SC_CONNECTABLE.value != device_info.status:
            print(" The first device [status]: {} does not support connection.".format(str(device_info.status)))
            exit(1)
    else:
        print("[scGetDeviceInfoList] fail, ScStatus({})".format(ret))
        exit(1) 
else: 
    print("there are no camera found")
    exit(1)

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print('[scOpenDeviceBySN] success ScStatus({}).'.format(str(ret)))
else:
    print('[scOpenDeviceBySN] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret, frameRate = camera.scGetFrameRate()
if  ret == 0:
    print('[scGetFrameRate] success ScStatus({}). The device frame rate is {}.'.format(str(ret), frameRate))
else:
    print('[scGetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1) 
#Get the result form AI module.
ResultCount = 100
for i in range(ResultCount):
    ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
    if ret != 0:
        print('[scAIModuleGetResult] fail ScStatus({}).'.format(str(ret)))
        time.sleep(0.005)
        continue
    ret, detecList = parse_ai_result(result)
    for i, res in enumerate(detecList):
        if res.isWarning == 1:
            print(f"area {res.index} is dangerous.")

ret = camera.scStopStream()
if  ret == 0:
    print('[scStopStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStopStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scCloseDevice()
if ret == 0:
    print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
else:
    print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))
    exit(1) 

exit(0) 