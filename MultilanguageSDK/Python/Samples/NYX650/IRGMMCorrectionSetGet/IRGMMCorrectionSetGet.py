﻿from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from API.ScepterDS_api import *
import time

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
	print("connect statu:",device_info.status)  
	print("Call scOpenDeviceBySN with connect status :",ScConnectStatus.SC_CONNECTABLE.value)
	exit()
else:
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print("open device successful")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))   

ret,params = camera.scGetIRGMMCorrection()
if  ret == 0:
    print("The default IRGMMCorrection switch is " + str(params.enable))
else:
    print("scGetIRGMMCorrection failed:"+ str(ret))

params.enable = not params.enable
ret = camera.scSetIRGMMCorrection(params)
if  ret == 0:
    print("Set IRGMMCorrection switch to "+ str(params.enable) + " is Ok")
else:
    print("scSetIRGMMCorrection failed:"+ str(ret))

ret = camera.scStartStream()
if ret == 0:
    print("start stream successful")
else:
    print("scStartStream failed:" + str(ret))

ret = camera.scStopStream()
if  ret == 0:
    print("stopstream success")
else:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret == 0:
    print("close device successful")
else:
    print('scCloseDevice failed: ' + str(ret)) 
    
print('Test end, please reboot camera to restore the default settings')  