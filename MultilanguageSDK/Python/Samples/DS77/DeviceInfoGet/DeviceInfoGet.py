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
    print('cam serialNumber:' + str(device_info.serialNumber))
    print('cam ip:' + str(device_info.ip))
    print('cam status:' + str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print("scOpenDeviceBySN")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))   
 
 
# get FW
ret, fw = camera.scGetFirmwareVersion()
if  ret == 0:
    print("scGetFirmwareVersion :",str(fw))
else:
    print("scGetFirmwareVersion failed status:",ret)

 
ret = camera.scCloseDevice()
if  ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret))   
           