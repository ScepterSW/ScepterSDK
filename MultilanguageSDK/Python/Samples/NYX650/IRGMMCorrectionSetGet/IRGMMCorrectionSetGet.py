from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---IRGMMCorrectionSetGet---")
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

ret,params = camera.scGetIRGMMCorrection()
if  ret == 0:
    if params.enable == 1:
        print('[scGetIRGMMCorrection] success ScStatus({}). The device default IR GMM Correction enable is {}, threshold is {}'.format(str(ret), str(params.enable).lower(), str(params.threshold)))
    else:
        print('[scGetIRGMMCorrection] success ScStatus({}). The device default IR GMM Correction enable is {}'.format(str(ret), str(params.enable).lower()))
else:
    print('[scGetIRGMMCorrection] fail ScStatus({}).'.format(str(ret)))
    exit(1)

params.enable = not params.enable
if params.enable == 1:
    params.threshold =  int(params.threshold / 2) + 30

ret = camera.scSetIRGMMCorrection(params)
if  ret == 0:
    if params.enable == 1:
        print('[scSetIRGMMCorrection] success ScStatus({}). Set the device IR GMM correction enable is {}, threshold is {}'.format(str(ret), str(params.enable).lower(), str(params.threshold)))
    else:
        print('[scSetIRGMMCorrection] success ScStatus({}). Set the device IR GMM correction enable is {}'.format(str(ret), str(params.enable).lower()))
else:
    print('[scSetIRGMMCorrection] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scCloseDevice()
if  ret == 0:
    print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
else:
    print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))
    exit(1)
    


exit(0)