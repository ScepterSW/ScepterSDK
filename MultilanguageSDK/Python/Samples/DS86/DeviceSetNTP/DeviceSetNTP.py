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
    print('cam serialNumber:' + str(device_info.serialNumber))
    print('cam ip:' + str(device_info.ip))
    print('cam status:' + str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if ret == 0:
    print("scOpenDeviceBySN")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))

# Set NTP
strIP = r"192.168.1.100"
params = ScTimeSyncConfig(flag = 1, ip = (c_uint8 * 16)(*bytes(strIP, 'utf-8')))
ret = camera.scSetRealTimeSyncConfig(params)
if ret == 0:
    print("Set NTP:" + strIP)
else:
    print("Set NTP failed status:" + str(ret))

ret = camera.scCloseDevice()
if ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret))   
           
