﻿from pickle import FALSE, TRUE
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

camera = ScepterTofCam()

camera_count = camera.scGetDeviceCount(3000)
print("Get device count:", camera_count)
if camera_count <= 0:
    print("scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.")
    exit()

device_info=ScDeviceInfo()

if camera_count > 0:
    ret,device_infolist=camera.scGetDeviceInfoList(camera_count)
    if ret==0:
        device_info = device_infolist[0]
    else:
        print(' failed:' + ret)  
        exit()  
else: 
    print("there are no camera found")
    exit()

if  ScConnectStatus.SC_CONNECTABLE.value != device_info.status:
	print("connect status:",device_info.status)
	print("Call scOpenDeviceBySN with connect status :",ScConnectStatus.SC_CONNECTABLE.value)
	exit()
else:
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print("scOpenDeviceBySN")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))
    exit()

ret = camera.scStartStream()
if  ret == 0:
    print("scStartStream successful")
else:
    print("scStartStream failed:"+ str(ret))
    exit()

ret = camera.scSetFrameRate(5)
if ret == 0:
    print("Set frame rate:5")
else:
    print("scSetFrameRate failed status:"+str(ret))
    exit()

ret,enabled = camera.scGetHDRModeEnabled()
if ret == 0:
    print("Get HDRMode status:"+str(enabled))
else:
    print("scGetHDRModeEnabled failed status:"+str(ret))
    exit()

if not enabled:
    ret,exposureControlMode  = camera.scGetExposureControlMode(ScSensorType.SC_TOF_SENSOR)
    if ret == 0:
        print("Get exposure control mode:"+str(exposureControlMode))
    else:
        print("scGetExposureControlMode failed status:"+str(ret))
        exit()
    if exposureControlMode == ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO:
        ret = camera.scSetExposureControlMode(ScSensorType.SC_TOF_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL)
        if ret == 0:
            print("Set exposure control manual mode")
        else:
            print("scSetExposureControlMode failed status:"+str(ret))
            exit()
    ret = camera.scSetHDRModeEnabled(True)
    if ret == 0:
        print("Open HDR mode")
    else:
        print("scSetHDRModeEnabled failed status:"+str(ret))
        exit()

ret,nCount = camera.scGetFrameCountOfHDRMode()
if 0 == ret:
    for i in range(nCount):
        ret,maxExposureTime = camera.scGetMaxExposureTimeOfHDR(i)
        if ret == 0:
            print("Set FrameCount: "+str(i)+" max exposure time:" + str(maxExposureTime))
        else:
            print("scGetMaxExposureTimeOfHDR FrameCount: " + str(i) + " failed status:"+str(ret))
            exit()
        ret,curExposureTime = camera.scGetExposureTimeOfHDR(i)
        if ret == 0:
            print("Get FrameCount: "+str(i)+" current exposure time:" + str(curExposureTime))
        else:
            print("scGetExposureTimeOfHDR FrameCount: " + str(i) + " failed status:"+str(ret))
            exit()

        nExposureTime = round(maxExposureTime / 2)
        ret = camera.scSetExposureTimeOfHDR(i, c_int32(nExposureTime))
        if ret == 0:
            print("Set FrameCount: "+str(i)+" exposure time:" + str(maxExposureTime/2))
        else:
            print("scSetExposureTimeOfHDR FrameCount: " + str(i) + " failed status:"+str(ret))
            exit()
else:
    print("scGetFrameCountOfHDRMode failed status:"+str(ret))
    exit()

ret = camera.scStopStream()
if  ret == 0:
    print("stopstream success")
else:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret)) 

print('Test end')