from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---DeviceParamSetGet---")
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

ret, params = camera.scGetSensorIntrinsicParameters(ScSensorType.SC_TOF_SENSOR)
if  ret == 0:
    print('[scGetSensorIntrinsicParameters] success ScStatus({}).'.format(str(ret)))
    print("Depth camera intinsic: \nFx: {}\nCx: {}\nFy: {}\nCy: {}".format(params.fx, params.cx, params.fy, params.cy))
    print("Depth distortion coefficient: \nK1: {}\nK2: {}\nP1: {}\nP2: {}\nK3: {}\nK4: {}\nK5: {}\nK6: {}".format(params.k1,params.k2,params.p1,params.p2,params.k3,params.k4,params.k5,params.k6))
else:
    print('[scGetSensorIntrinsicParameters] fail ScStatus({}).'.format(str(ret)))

ret, gmmgain = camera.scGetIRGMMGain()
if  ret == 0:
    print('[scGetIRGMMGain] success ScStatus({}). The device IRGMM gain is {}.'.format(str(ret), gmmgain))
else:
    print('[scGetIRGMMGain] fail ScStatus({}).'.format(str(ret)))
    exit(1)

gmmgain = 50
ret = camera.scSetIRGMMGain(gmmgain)
if  ret == 0:
    print('[scSetIRGMMGain] success ScStatus({}). Set the device IRGMM gain to {}.'.format(str(ret), gmmgain))
else:
    print('[scSetIRGMMGain] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scCloseDevice()
if  ret == 0:
    print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
else:
    print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))
    exit(1)

exit(0)
