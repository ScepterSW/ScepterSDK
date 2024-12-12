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
retry_count = 100
while camera_count < 2 and retry_count > 0:
    retry_count = retry_count-1
    camera_count = camera.scGetDeviceCount(1000)
    print("scaning......   ", retry_count)

print("Get device count:", camera_count)


if camera_count < 2: 
    print("there are no camera or only one camera")
    exit()

print("cam count :",camera_count)
cameras = []

ret, device_infolist=camera.scGetDeviceInfoList(camera_count)
if ret==0:
    for i in range(camera_count): 
        print('cam serialNumber:  ' + str(device_infolist[i].serialNumber))
        cam = ScepterTofCam()
        ret = cam.scOpenDeviceBySN(device_infolist[i].serialNumber)
        if  ret == 0:
            print(device_infolist[i].serialNumber,"open successful")
            cameras.append(cam)
        else:
            print(device_infolist[i].serialNumber,'scOpenDeviceBySN failed: ' + str(ret))    
else:
    print(' failed:' + ret)  
    exit()  

for i in range(camera_count): 
    ret = cameras[i].scStartStream()       
    if  ret == 0:
        print(device_infolist[i].serialNumber,"scStartStream successful")
    else:
        print(device_infolist[i].serialNumber,'scStartStream failed: ' + str(ret))  

# show image 

while 1:
    for i in range(camera_count): 
        ret, frameready = cameras[i].scGetFrameReady(c_uint16(1200))
        if  ret !=0:
            print("scGetFrameReady failed status:",ret)
            continue
                        
        if  frameready.depth:      
            ret,depthframe = cameras[i].scGetFrame(ScFrameType.SC_DEPTH_FRAME)
            if  ret == 0:
                print(device_infolist[i].serialNumber,"  depth frameindex: ",depthframe.frameIndex)
            else:
                print("scGetFrame error", ret)
        if  frameready.ir:
            ret,irframe = cameras[i].scGetFrame(ScFrameType.SC_IR_FRAME)
            if  ret == 0:
                print(device_infolist[i].serialNumber,"  ir frameindex: ",irframe.frameIndex)
            else:
                print("scGetFrame error", ret)

for i in range(camera_count): 
    
    ret = cameras[i].scStopStream()       
    if  ret == 0:
        print("stop stream successful")
    else:
        print('scStopStream failed: ' + str(ret))  

    ret = cameras[i].scCloseDevice()       
    if  ret == 0:
        print("scCloseDevice successful")
    else:
        print('scCloseDevice failed: ' + str(ret))  
    
           