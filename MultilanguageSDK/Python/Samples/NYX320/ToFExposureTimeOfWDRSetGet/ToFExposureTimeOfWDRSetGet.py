from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time

class ScExposureTimeParams(Structure):
    _pack_ = 1
    _fields_ = [("mode", c_uint32),
                ("exposureTime", c_int32)]
print("---ToFExposureTimeOfWDRSetGet---")
camera = ScepterTofCam()

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

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scSetFrameRate(5)
if ret == 0:
    print('[scSetFrameRate] success ScStatus({}). Set the device frame rate to 5'.format(str(ret)))
else:
    print('[scSetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret,enabled = camera.scGetWDRModeEnabled()
if ret == 0:
    print('[scGetWDRModeEnabled] success ScStatus({}). Get WDR mode status: {}'.format(str(ret), str(enabled).lower()))
else:
    print('[scGetWDRModeEnabled] fail ScStatus({}).'.format(str(ret)))
    exit(1)

if not enabled:
    ret,exposureControlMode  = camera.scGetExposureControlMode(ScSensorType.SC_TOF_SENSOR)
    if ret == 0:
        print('[scGetExposureControlMode] success ScStatus({}). Get exposure control mode: {}'.format(str(ret), str(exposureControlMode)))
    else:
        print('[scGetExposureControlMode] fail ScStatus({}).'.format(str(ret)))
        exit(1)
    if exposureControlMode == ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO:
        ret = camera.scSetExposureControlMode(ScSensorType.SC_TOF_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL)
        if ret == 0:
            print('[scSetExposureControlMode] success ScStatus({}). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success.'.format(str(ret)))
        else:
            print('[scSetExposureControlMode] fail ScStatus({}).'.format(str(ret)))
            exit(1)
    '''Set WDR mode enabled, HDR function need to be turned off in advance.'''
    ret = camera.scSetWDRModeEnabled(True)
    if ret == 0:
        print('[scSetWDRModeEnabled] success ScStatus({}). Enabled WDR mode.'.format(str(ret)))
    else:
        print('[scSetWDRModeEnabled] fail ScStatus({}).'.format(str(ret)))
        exit(1)

ret,nCount = camera.scGetFrameCountOfWDRMode()
if 0 == ret:
    print('[scGetFrameCountOfWDRMode] success ScStatus({}). Get WDR mode fame count: {}'.format(str(ret), str(nCount)))
    for i in range(nCount):
        ret,maxExposureTime = camera.scGetMaxExposureTimeOfWDR(i)
        if ret == 0:
            print('[scGetMaxExposureTimeOfWDR] success ScStatus({}). Get frame count: {} max WDR exposure time: {}'.format(str(ret), str(i), str(maxExposureTime)))
        else:
            print('[scGetMaxExposureTimeOfWDR] fail ScStatus({}).'.format(str(ret)))
            exit(1)
        ret,curExposureTime = camera.scGetExposureTimeOfWDR(i)
        if ret == 0:
            print('[scGetExposureTimeOfWDR] success ScStatus({}). Get frame count: {} current WDR exposure time: {}'.format(str(ret), str(i), str(curExposureTime)))
        else:
            print('[scGetExposureTimeOfWDR] fail ScStatus({}).'.format(str(ret)))
            exit(1)

        nExposureTime = round(maxExposureTime / 2)
        ret = camera.scSetExposureTimeOfWDR(i, c_int32(nExposureTime))
        if ret == 0:
            print('[scSetExposureTimeOfWDR] success ScStatus({}). Set frame count: {} WDR exposure time: {}'.format(str(ret), str(i), str(int(maxExposureTime/2))))
        else:
            print('[scSetExposureTimeOfWDR] fail ScStatus({}).'.format(str(ret)))
            exit(1)
else:
    print('[scGetFrameCountOfWDRMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

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

print('Test end')

exit(0)