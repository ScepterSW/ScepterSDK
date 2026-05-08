from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---SingleFrameDelayTest---")
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


ret = camera.scSetWorkMode(ScWorkMode.SC_SOFTWARE_TRIGGER_MODE)
if  ret == 0:
    print('[scSetWorkMode] success ScStatus({}). Set SC_SOFTWARE_TRIGGER_MODE.'.format(str(ret)))
else:
    print('[scSetWorkMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret, frameRate = camera.scGetFrameRate()
if  ret == 0:
    print('[scGetFrameRate] success ScStatus({}). The device frame rate is {}.'.format(str(ret), frameRate))
else:
    print('[scGetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1) 

curPath = os.getcwd()
filename = curPath + "/SingleFrameDelayTest.csv"
try:
    file = open(filename, "w")
except IOError:
    print('csv file open failed')
    exit(1)
file.write("frameIndex,TotalDelay,ExcludeDelayofExposure\n")

'''
//1.software trigger.
//2.ReadNextFrame.
//3.GetFrame acoording to Ready flag and Frametype.
//4.sleep 1000/frameRate (ms)
'''
number = 100
for i in range(number):
    ret = camera.scSoftwareTriggerOnce()
    startTime = time.perf_counter()
    if  ret != 0:  
        print('[scSoftwareTriggerOnce] fail ScStatus({}).'.format(str(ret)))
        continue

    ret, frameready = camera.scGetFrameReady(c_uint16(15000))
    if  ret != 0:
        print('[scGetFrameReady] fail ScStatus({}).'.format(str(ret)))
        continue
    if  frameready.depth:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
        if  ret == 0:
            if 0 == frame.frameIndex % 10:
                print('[scGetFrame] success ScStatus({}). SC_DEPTH_FRAME <frameIndex>: {}'.format(str(ret), str(frame.frameIndex)))
            endTime = time.perf_counter()
            endTimeUtc = int(time.time()*1000)
            exposureEndTime = frame.hardwaretimestamp
            frameIntervalTime = endTime - startTime
            frameIntervalNtpTime = endTimeUtc - exposureEndTime
            if frameIntervalNtpTime > 2000:
                file.write("{0},{1},N/A\n".format(frame.frameIndex, int(frameIntervalTime*1000)))
            else:
                file.write("{0},{1},{2}\n".format(frame.frameIndex, int(frameIntervalTime*1000), frameIntervalNtpTime))
        else:
            print('[scGetFrame] fail ScStatus({}).'.format(str(ret)))  
    time.sleep(1/frameRate)
file.close()

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret == 0:
    print('[scSetWorkMode] success ScStatus({}). Set SC_ACTIVE_MODE.'.format(str(ret)))
else:
    print('[scSetWorkMode] fail ScStatus({}).'.format(str(ret)))
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