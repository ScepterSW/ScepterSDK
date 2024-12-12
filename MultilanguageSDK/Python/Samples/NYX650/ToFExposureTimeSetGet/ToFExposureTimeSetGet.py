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

'''Testing TOF exposure time requires turning off HDR and WDR in advance.'''
ret = camera.scSetExposureControlMode()
if  ret == 0:
    print("set tofsensor to manual mode successful")
else:
    print("scSetExposureControlMode failed:"+ str(ret))   

ret,frameRate = camera.scGetFrameRate()
if  ret == 0:
    print("Get default frame rate:"+ str(frameRate))   
else:
    print("scGetFrameRate failed:"+ str(ret))   

ret,MaxExposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_TOF_SENSOR)
if  ret == 0:
    print("Recommended scope: 58 - "+ str(MaxExposureTime))   
else:
    print("scGetMaxExposureTime failed:"+ str(ret))   

ret = camera.scSetExposureTime(ScSensorType.SC_TOF_SENSOR,400)
if  ret == 0:
    print("Set exposure time 400 is ok")   
else:
    print("scSetExposureTime failed:"+ str(ret))   

ret = camera.scSetFrameRate(5)
if  ret == 0:
    print("Set frame rate 5 is ok")   
else:
    print("scSetFrameRate failed:"+ str(ret))   

ret,MaxExposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_TOF_SENSOR)
if  ret == 0:
    print("Recommended scope: 58 - "+ str(MaxExposureTime))   
else:
    print("scGetMaxExposureTime failed:"+ str(ret))   

ret = camera.scSetExposureTime(ScSensorType.SC_TOF_SENSOR,500)
if  ret == 0:
    print("Set exposure time 500 is ok")   
else:
    print("scSetExposureTime failed:"+ str(ret))   
	
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