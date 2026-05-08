from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---MultiConnection---")
camera = ScepterTofCam()

camera_count = 0
while camera_count < 2:
    camera_count = camera.scGetDeviceCount(3000)
    print("[scGetDeviceCount] success, ScStatus(0), The device count is {}".format(camera_count))

cameras = []

ret, device_infolist=camera.scGetDeviceInfoList(camera_count)
if ret==0:
    print("[scGetDeviceInfoList] success, ScStatus({})".format(ret))
    for i in range(camera_count): 
        print(" The device index {}, <serialNumber>: {} , <ip>: {} , <status>: {}".format(str(i), str(device_infolist[i].serialNumber.decode()), str(device_infolist[i].ip.decode()), str(device_infolist[i].status)))
        cam = ScepterTofCam()
        ret = cam.scOpenDeviceBySN(device_infolist[i].serialNumber)
        if  ret == 0:
            print('[scOpenDeviceBySN] success ScStatus({}). The device index {} , <serialNumber>: {}'.format(str(ret), str(i), str(device_infolist[i].serialNumber.decode())))
            cameras.append(cam)
        else:
            print('[scOpenDeviceBySN] fail ScStatus({}). The device index {} , <serialNumber>: {}'.format(str(ret), str(i), str(device_infolist[i].serialNumber.decode())))
            exit(1)
else:
    print("[scGetDeviceInfoList] fail, ScStatus({})".format(ret))
    exit(1)  

for i in range(camera_count): 
    ret = cameras[i].scStartStream()       
    if  ret == 0:
        print('[scStartStream] success ScStatus({}).  The device index {}'.format(str(ret), str(i)))
    else:
        print('[scStartStream] fail ScStatus({}).  The device index {}'.format(str(ret), str(i)))
        exit(1)

#Wait for the device to upload image data.
time.sleep(1)

# show image 

for ind in range(10):
    for i in range(camera_count): 
        ret, frameready = cameras[i].scGetFrameReady(c_uint16(1200))
        if  ret != 0:
            print('[scGetFrameReady] fail ScStatus({}).'.format(str(ret)))
            continue
        if  frameready.depth:      
            ret,depthframe = cameras[i].scGetFrame(ScFrameType.SC_DEPTH_FRAME)
            if  ret == 0:
                print('[scGetFrame] success ScStatus({}). The device index: {}, SC_DEPTH_FRAME <frameIndex>: {}'.format(str(ret), str(i), str(depthframe.frameIndex)))
            else:
                print('[scGetFrame] fail ScStatus({}). The device index: {}, SC_DEPTH_FRAME <frameIndex>: {}'.format(str(ret), str(i), str(depthframe.frameIndex)))
        if  frameready.ir:
            ret,irframe = cameras[i].scGetFrame(ScFrameType.SC_IR_FRAME)
            if  ret == 0:
                print('[scGetFrame] success ScStatus({}). The device index: {}, SC_IR_FRAME <frameIndex>: {}'.format(str(ret), str(i), str(irframe.frameIndex)))
            else:
                print('[scGetFrame] fail ScStatus({}). The device index: {}, SC_IR_FRAME <frameIndex>: {}'.format(str(ret), str(i), str(irframe.frameIndex)))

for i in range(camera_count): 
    
    ret = cameras[i].scStopStream()       
    if  ret == 0:
        print('[scStopStream] success ScStatus({}). The device index: {}'.format(str(ret), str(i)))
    else:
        print('[scStopStream] fail ScStatus({}). The device index: {}'.format(str(ret), str(i)))
        exit(1)

    ret = cameras[i].scCloseDevice()       
    if  ret == 0:
        print('[scCloseDevice] success ScStatus({}). The device index: {}'.format(str(ret), str(i)))
    else:
        print('[scCloseDevice] fail ScStatus({}). The device index: {}'.format(str(ret), str(i)))
        exit(1)

exit(0)