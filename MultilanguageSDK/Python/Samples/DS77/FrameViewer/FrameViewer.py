from pickle import FALSE, TRUE
import sys
import numpy
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import cv2
import time
print("---FrameViewer---")
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
    ret = camera.scStartStream()
    if  ret == 0:
        print('[scStartStream] success ScStatus({}).'.format(str(ret)))
        #Wait for the device to upload image data.
        time.sleep(1)
    else:
        print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
        exit(1) 

    colorSlope = c_uint16(7495)
    
    try:
        while 1:

            ret, frameready = camera.scGetFrameReady(c_uint16(15000))
            if  ret != 0:
                print('[scGetFrameReady] fail ScStatus({}).'.format(str(ret)))
                continue
            hasDepth=0
            hasIR =0
            if  frameready.depth:      
                ret,depthframe = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
                if  ret == 0:
                    hasDepth=1
                else:
                    print('[scGetFrame] depth fail ScStatus({}).'.format(str(ret)))  

            if  frameready.ir:
                ret,irframe = camera.scGetFrame(ScFrameType.SC_IR_FRAME)
                if  ret == 0:
                    hasIR =1
                else:
                    print('[scGetFrame] ir fail ScStatus({}).'.format(str(ret)))  

            if  hasDepth==1:
                frametmp = numpy.ctypeslib.as_array(depthframe.pFrameData, (1, depthframe.width * depthframe.height * 2))
                frametmp.dtype = numpy.uint16
                frametmp.shape = (depthframe.height, depthframe.width)

                #convert ushort value to 0xff is just for display
                img = numpy.int32(frametmp)
                img = img*255/colorSlope
                img = numpy.clip(img, 0, 255)
                img = numpy.uint8(img)
                frametmp = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)

                cv2.imshow("Depth Image", frametmp)

            if  hasIR==1:
                frametmp = numpy.ctypeslib.as_array(irframe.pFrameData, (1, irframe.dataLen))
                frametmp.dtype = numpy.uint8
                frametmp.shape = (irframe.height, irframe.width)
                    
                cv2.imshow("IR Image", frametmp)

            key = cv2.waitKey(1)
            if  key == 27:
                cv2.destroyAllWindows()
                break
    except Exception as e :
        print(e)
    finally :
        print('end')
else:
    print('[scOpenDeviceBySN] fail ScStatus({}).'.format(str(ret)))
    exit(1)

exit(0)