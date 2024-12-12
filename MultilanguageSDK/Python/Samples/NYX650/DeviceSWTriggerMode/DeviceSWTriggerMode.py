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
if  ret != 0:
    print('scOpenDeviceBySN failed: ' + str(ret))
    exit()
	
print("scOpenDeviceBySN,status :"+ str(ret))

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed:"+ str(ret))
    exit()   

ret = camera.scSetWorkMode(ScWorkMode.SC_SOFTWARE_TRIGGER_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)

frameRate = camera.scGetFrameRate()
print("scGetFrameRate :",frameRate[1])

for i in range(30):

    ret = camera.scSoftwareTriggerOnce()
    if  ret != 0:  
        print("scSoftwareTriggerOnce failed status:",ret)

    ret, frameready = camera.scGetFrameReady(c_uint16(15000))
    if  ret !=0:
        print("scGetFrameReady failed status:",ret)
        continue       
    if  frameready.depth:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
        if  ret == 0:
            print("depth  id:",frame.frameIndex)  
        else:   
            print("depth  error:",ret)   
    time.sleep(1/frameRate[1])

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)
    
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
           