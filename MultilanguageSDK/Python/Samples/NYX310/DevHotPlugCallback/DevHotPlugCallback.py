from pickle import FALSE, TRUE
import sys
currentPath = sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---DevHotPlugCallbackC---")
camera = ScepterTofCam()

def HotPlugStateCallback(type_struct,  state = c_int32(0)):
    print("\n")
    global camera
    if state ==0:
        print("The device is <Added> deviceInfo <serialNumber>: {}, <ip>: {}, <status>: {}".format(str(type_struct.contents.serialNumber.decode()), str(type_struct.contents.ip.decode()), str(type_struct.contents.status)))
        ret = camera.scOpenDeviceBySN(type_struct.contents.serialNumber)
        if  ret == 0:
            print('[scOpenDeviceBySN] success ScStatus({}).'.format(str(ret)))
        else:
            print('[scOpenDeviceBySN] fail ScStatus({}).'.format(str(ret)))
        ret = camera.scStartStream()
        if  ret == 0:
            print('[scStartStream] success ScStatus({}).'.format(str(ret)))
        else:
            print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    else:
        print("The device is <Renmoved> deviceInfo <serialNumber>: {}, <ip>: {}, <status>: {}".format(str(type_struct.contents.serialNumber.decode()), str(type_struct.contents.ip.decode()), str(type_struct.contents.status)))
        ret = camera.scStopStream()
        if  ret == 0:
            print('[scStopStream] success ScStatus({}).'.format(str(ret)))
        else:
            print('[scStopStream] fail ScStatus({}).'.format(str(ret)))
        ret = camera.scCloseDevice()
        if  ret == 0:
            print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
        else:
            print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))

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
    else:
        print("[scGetDeviceInfoList] fail, ScStatus({})".format(ret))
        exit(1)  
else: 
    print("there are no camera found")
    exit(1)

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print('[scOpenDeviceBySN] success ScStatus({}).'.format(str(ret)))
    ret = camera.scStartStream()
    if  ret == 0:
        print('[scStartStream] success ScStatus({}).'.format(str(ret)))
    else:
        print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
        exit(1)

    ret = camera.scSetHotPlugStatusCallback(HotPlugStateCallback)
    if ret==0:
        print("[scSetHotPlugStatusCallback] success, ScStatus({}). Wait for hotplug operation".format(ret))
        while 1:
            time.sleep(1)
    else:
        print("[scSetHotPlugStatusCallback] fail, ScStatus({})".format(ret))
        exit(1)
else:
    print('[scOpenDeviceBySN] fail ScStatus({}).'.format(str(ret)))
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

exit(0)

