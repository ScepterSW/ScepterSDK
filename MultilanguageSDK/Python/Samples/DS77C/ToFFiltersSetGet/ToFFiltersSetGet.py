from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---ToFFiltersSetGet---")
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

print("\n---1. Test TimeFilter---")
ret,params = camera.scGetTimeFilterParams()
if  ret == 0:
    print('[scGetTimeFilterParams] success ScStatus({}). The default time filter switch is {} .'.format(str(ret), str(params.enable)))
else:
    print('[scGetTimeFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

params.enable = not params.enable
ret = camera.scSetTimeFilterParams(params)
if  ret == 0:
    print('[scSetTimeFilterParams] success ScStatus({}). Set TimeFilter switch to {} is OK.'.format(str(ret), str(params.enable)))
else:
    print('[scSetTimeFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("\n---2. Test ConfidenceFilter---")
ret,params = camera.scGetConfidenceFilterParams()
if  ret == 0:
    print('[scGetConfidenceFilterParams] success ScStatus({}). The default ConfidenceFilter switch is {} .'.format(str(ret), str(params.enable)))
else:
    print('[scGetConfidenceFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

params.enable = not params.enable
ret = camera.scSetConfidenceFilterParams(params)
if  ret == 0:
    print('[scSetConfidenceFilterParams] success ScStatus({}). Set ConfidenceFilter switch to {} is OK.'.format(str(ret), str(params.enable)))
else:
    print('[scSetConfidenceFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("\n---3. Test FlyingPixelFilter---")
ret,params = camera.scGetFlyingPixelFilterParams()
if  ret == 0:
    print('[scGetFlyingPixelFilterParams] success ScStatus({}). The default flying pixel filter switch is {}.'.format(str(ret), str(params.enable)))
else:
    print('[scGetFlyingPixelFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

params.enable = not params.enable
ret = camera.scSetFlyingPixelFilterParams(params)
if  ret == 0:
    print('[scSetFlyingPixelFilterParams] success ScStatus({}). Set flying pixel filter switch to {} is OK.'.format(str(ret), str(params.enable)))
else:
    print('[scSetFlyingPixelFilterParams] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("\n---4. Test FillHoleFilter---")
ret,enable = camera.scGetFillHoleFilterEnabled()
if  ret == 0:
    print('[scGetFillHoleFilterEnabled] success ScStatus({}). The default fill hole filter switch is {}.'.format(str(ret), str(enable)))
else:
    print('[scGetFillHoleFilterEnabled] fail ScStatus({}).'.format(str(ret)))
    exit(1)

enable = not enable
ret = camera.scSetFillHoleFilterEnabled(enable)
if  ret == 0:
    print('[scSetFillHoleFilterEnabled] success ScStatus({}). Set flying pixel filter switch to {} is OK.'.format(str(ret), str(enable)))
else:
    print('[scSetFillHoleFilterEnabled] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("\n---5. Test SpatialFilter---")
ret,enable = camera.scGetSpatialFilterEnabled()
if  ret == 0:
    print('[scGetSpatialFilterEnabled] success ScStatus({}). The default spatial filter switch is {}.'.format(str(ret), str(enable)))
else:
    print('[scGetSpatialFilterEnabled] fail ScStatus({}).'.format(str(ret)))
    exit(1)

enable = not enable
ret = camera.scSetSpatialFilterEnabled(enable)
if  ret == 0:
    print('[scSetSpatialFilterEnabled] success ScStatus({}). Set spatial filter switch to {} is OK.'.format(str(ret), str(enable)))
else:
    print('[scSetSpatialFilterEnabled] fail ScStatus({}).'.format(str(ret)))
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