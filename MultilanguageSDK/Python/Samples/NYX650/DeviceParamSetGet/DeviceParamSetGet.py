from pickle import FALSE, TRUE
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
if  ret != 0:
    print('scOpenDeviceBySN failed: ' + str(ret))
    exit()
	
print("open device successful,status :"+ str(ret))

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed:"+ str(ret))
    exit()     


ret, params = camera.scGetSensorIntrinsicParameters(ScSensorType.SC_TOF_SENSOR)
if  ret == 0:
    print("scGetSensorIntrinsicParameters SC_TOF_SENSOR :",
    params.fx,
    params.fy,
    params.cx,
    params.cy, 
    params.k1,
    params.k2,
    params.p1,
    params.p2,
    params.k3,
    params.k4,
    params.k5,
    params.k6)
else:
    print("scGetSensorIntrinsicParameters SC_TOF_SENSOR failed:",ret)

ret, gmmgain = camera.scGetIRGMMGain()
if  ret == 0:
    print("scGetIRGMMGain :",gmmgain)
else:
    print("scGetIRGMMGain failed:",ret)

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
           
