from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time

camera = ScepterTofCam()

print("---SingleFrameDelayTest---")
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
        print('GetDeviceListInfo failed status:' + ret)
        exit()  
else: 
    print("there are no camera found")
    exit()

if  ScConnectStatus.SC_CONNECTABLE.value != device_info.status:
	print("connect status:",device_info.status)  
	print("The device state does not support connection.")
	exit()
else:
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret != 0:
    print('OpenDevice failed status:' + str(ret))
    exit()


ret = camera.scSetWorkMode(ScWorkMode.SC_SOFTWARE_TRIGGER_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)

ret, frameRate = camera.scGetFrameRate()
if ret == 0:
    print("frameRate :", frameRate)
else:
    print("scGetFrameRate :", ret)
    exit()

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed status:"+ str(ret))
    exit()

curPath = os.getcwd()
filename = curPath + "/SingleFrameDelayTest.csv"
try:
    file = open(filename, "w")
except IOError:
    print('csv file open failed')
    exit()
file.write("frameIndex,TotalDelay,ExcludeDelayofExposure\n")

'''
//1.software trigger.
//2.ReadNextFrame.
//3.GetFrame acoording to Ready flag and Frametype.
//4.sleep 1000/frameRate (ms)
'''
number = int(input('Please input the number of tests:'))
for i in range(number):
    ret = camera.scSoftwareTriggerOnce()
    startTime = time.perf_counter()
    if  ret != 0:  
        print("scSoftwareTriggerOnce failed:",ret)

    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if  ret != 0:
        print("scGetFrameReady failed status:",ret)
        continue       
    if  frameready.depth:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
        if  ret == 0:
            if 0 == frame.frameIndex % 10:
                print("scGetFrame,status:", ret, "  ", "frameType:", frame.frameType, "  ", "frameIndex:", frame.frameIndex)
            endTime = time.perf_counter()
            endTimeUtc = int(time.time()*1000)
            exposureEndTime = frame.hardwaretimestamp
            frameIntervalTime = endTime - startTime
            frameIntervalNtpTime = endTimeUtc - exposureEndTime
            if frameIntervalNtpTime > 2000:
                file.write("{0},{1},N/A\n".format(frame.frameIndex, int(frameIntervalTime*1000)))
            else:
                file.write("{0},{1},{2}\n".format(frame.frameIndex, int(frameIntervalTime*1000), frameIntervalNtpTime))
    time.sleep(1/frameRate)
file.close()

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)

ret = camera.scStopStream()
if  ret != 0:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret != 0:
    print('scCloseDevice failed: ' + str(ret))
           