from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

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

ret = camera.scStartStream()
if  ret == 0:
    print("scStartStream successful")
else:
    print("scStartStream failed:"+ str(ret))   
 
ret,params = camera.scGetTimeFilterParams()
if  ret == 0:
    print("The default TimeFilter switch is " + str(params.enable))
else:
    print("scGetTimeFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.scSetTimeFilterParams(params)
if  ret == 0:
    print("Set TimeFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("scSetTimeFilterParams failed:"+ str(ret))   

ret,params = camera.scGetConfidenceFilterParams()
if  ret == 0:
    print("The default ConfidenceFilter switch is " + str(params.enable))
else:
    print("scGetConfidenceFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.scSetConfidenceFilterParams(params)
if  ret == 0:
    print("Set ConfidenceFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("scSetConfidenceFilterParams failed:"+ str(ret))   

ret,params = camera.scGetFlyingPixelFilterParams()
if  ret == 0:
    print("The default FlyingPixelFilter switch is " + str(params.enable))
else:
    print("scGetFlyingPixelFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.scSetFlyingPixelFilterParams(params)
if  ret == 0:
    print("Set FlyingPixelFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("scSetFlyingPixelFilterParams failed:"+ str(ret))   

ret,enable = camera.scGetFillHoleFilterEnabled()
if  ret == 0:
    print("The default FillHoleFilter switch is " + str(enable))
else:
    print("scGetFillHoleFilterEnabled failed:"+ str(ret))   

enable = not enable
ret = camera.scSetFillHoleFilterEnabled(enable)
if  ret == 0:
    print("Set FillHoleFilter switch to "+ str(enable) + " is Ok")   
else:
    print("scSetFillHoleFilterEnabled failed:"+ str(ret))   

ret,enable = camera.scGetSpatialFilterEnabled()
if  ret == 0:
    print("The default SpatialFilter switch is " + str(enable))
else:
    print("scGetSpatialFilterEnabled failed:"+ str(ret))   

enable = not enable
ret = camera.scSetSpatialFilterEnabled(enable)
if  ret == 0:
    print("Set SpatialFilter switch to "+ str(enable) + " is Ok")   
else:
    print("scSetSpatialFilterEnabled failed:"+ str(ret))   

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
    
print('Test end, please reboot camera to restore the default settings')  