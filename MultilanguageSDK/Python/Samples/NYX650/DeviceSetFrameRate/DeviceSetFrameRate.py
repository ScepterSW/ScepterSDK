from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---DeviceSetFrameRate---")
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

ret = camera.scSetFrameRate(5)
if ret == 0:
    print('[scSetFrameRate] success ScStatus({}). Set frame rate to 5.'.format(str(ret)))
else:
    print('[scSetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret == 0:
    print('[scSetWorkMode] success ScStatus({}). Set SC_ACTIVE_MODE.'.format(str(ret)))
else:
    print('[scSetWorkMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1) 
print("Start testing the average frame rate for 30 seconds, please wait patiently.")
TESTPERIOD = 30
index = 0
start = 0
diff = 0
while 1:
    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if  ret == 0:
        print('[scGetFrameReady] success ScStatus({}).'.format(str(ret)))
    else:
        print('[scGetFrameReady] fail ScStatus({}).'.format(str(ret)))
        continue 

    if frameready.depth:
        ret, frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
        if ret == 0:
            print('[scGetFrame] success ScStatus({}). SC_DEPTH_FRAME <frameIndex>: {}'.format(str(ret), str(frame.frameIndex)))
            if start == 0:
                start = int(time.time()*1000)
            diff = int(time.time()*1000) - start
            index += 1
            if diff > TESTPERIOD * 1000:
                fps = index / (diff / 1000)
                print("Average frame rate: ", fps)
                break
        else:
            print('[scGetFrame] fail ScStatus({}).'.format(str(ret)))

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