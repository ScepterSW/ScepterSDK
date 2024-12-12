from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import cv2
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
        print(' failed:' , ret)  
        exit()
else: 
    print("there are no camera found")
    exit()

print("serialNumber: "+str(device_info.serialNumber))
ret = camera.scOpenDeviceBySN(device_info.serialNumber)

if  ret == 0:

    ret = camera.scStartStream()
    if  ret == 0:
        print("scStartStream successful")
    else:
        print("scStartStream failed status:",ret)

 
    colorSlope = c_uint16(7495)
    
    try:
        while 1:

            ret, frameready = camera.scGetFrameReady(c_uint16(1200))
            if  ret !=0:
                print("scGetFrameReady failed status:",ret)
                continue
            hasDepth=0
            hasIR =0
            
            if  frameready.depth:      
                ret,depthframe = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
                if  ret == 0:
                    hasDepth=1
                   
                else:
                    print("get depth frame failed status:",ret)
 
            if  frameready.ir:
                ret,irframe = camera.scGetFrame(ScFrameType.SC_IR_FRAME)
                if  ret == 0:
                    hasIR =1
                  
                else:
                    print("get ir frame failed status:",ret)
 
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
                print("---end---")
                break;
                   
    except Exception as e :
        print(e)
    finally :
        print('end')
else:
    print('scOpenDeviceBySN failed: ' + str(ret))
