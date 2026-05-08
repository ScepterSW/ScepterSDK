from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
print(libpath + "API")
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---AISingleRunMode---")
camera = ScepterTofCam()

def setParameters():
    # Set AI module continuous running.
    ret = camera.scAIModuleSetWorkMode(ScAIModuleMode.AI_SINGLE_RUN_MODE)
    if ret == 0:
        print("[scAIModuleSetWorkMode] success, ScStatus({}). Set AI_SINGLE_RUN_MODE.".format(ret))
    else:
        print("[scAIModuleSetWorkMode] fail, ScStatus({})".format(ret))
        return ret

	# Enable AI module.
    ret = camera.scAIModuleSetEnabled(c_bool(True))
    if ret == 0:
        print("[scAIModuleSetEnabled] success, ScStatus({}). Enable AI module.".format(ret))
    else:
        print("[scAIModuleSetEnabled] fail, ScStatus({})".format(ret))
        return ret
    return 0

camera_count = camera.scGetDeviceCount(3000)
print("[scGetDeviceCount] success, ScStatus(0), The device count is {}".format(camera_count))
if camera_count <= 0:
    print("[scGetDeviceCount] scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.")
    exit(1)

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

ret = setParameters()
if  ret == 0:
    print('[setParameters] success ScStatus({}).'.format(str(ret)))
else:
    print('[setParameters] fail ScStatus({}).'.format(str(ret)))
    exit(1)

pbuffer = ctypes.c_void_p()
pbufferSize = ctypes.c_uint16(0)
ret = camera.scAIModuleGetParam(paramID = c_uint32(0), pBuffer = pbuffer, bufferSize = pbufferSize)
if ret == 0:
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("[scAIModuleGetParam] success ScStatus({}). The algo's version is: {}".format(str(ret), str(str_buffer)))
else:
    print('[scAIModuleGetParam] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scAIModuleGetParam(paramID = c_uint32(1), pBuffer = pbuffer, bufferSize = pbufferSize)
if ret == 0:
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("[scAIModuleGetParam] success ScStatus({}). The algo's name is: {}".format(str(ret), str(str_buffer)))
else:
    print('[scAIModuleGetParam] fail ScStatus({}).'.format(str(ret)))
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
ResultCount = 10
for i in range(ResultCount):
    # Call the below api to trigger one result, then the result will be sent.
    # If do not call this function, the result will not be sent and the API of scAIModuleGetResult will return timeout fail.
    ret = camera.scAIModuleTriggerOnce()
    if ret == 0:
        print('[scAIModuleTriggerOnce] success ScStatus({}).'.format(str(ret)))
    else:
        print('[scAIModuleTriggerOnce] fail ScStatus({}).'.format(str(ret)))
        continue
    ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
    if ret == 0:
        print('[scAIModuleGetResult] success ScStatus({}). ScAIResult <resultIndex>: {}'.format(str(ret), str(result.resultIndex)))
    else:
        print('[scAIModuleGetResult] fail ScStatus({}).'.format(str(ret)))
        time.sleep(0.005)
        continue
    time.sleep(1.2/frameRate)

ret = camera.scStopStream()
if  ret == 0:
    print('[scStopStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStopStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scCloseDevice()
if  ret == 0:
    print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
else:
    print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))
    exit(1)

exit(0)