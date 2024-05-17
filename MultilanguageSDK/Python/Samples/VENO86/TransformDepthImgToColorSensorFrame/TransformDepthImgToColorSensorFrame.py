from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from API.ScepterDS_api import *
import time

frameSpace = 20
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
	print("connect statu:",device_info.status)  
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
	
print("open device successful,status :"+ str(ret))

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed:"+ str(ret))
    exit()     

# set Mapper
ret = camera.scSetTransformDepthImgToColorSensorEnabled(c_bool(True))

if  ret == 0:
    print("scSetTransformDepthImgToColorSensorEnabled ok")
else:
    print("scSetTransformDepthImgToColorSensorEnabled failed:",ret)     

for i in range(frameSpace):
    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if  ret !=0:
        print("scGetFrameReady failed:",ret)
        continue       
    
    if  frameready.transformedDepth:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
        if  ret == 0:
            print("get Frame successful,status:" , ret , "  "
					,"frameTpye:" , frame.frameType , "  "
					,"frameIndex:" , frame.frameIndex )
        else:   
            print("transformedDepth  error:",ret)  

ret = camera.scStopStream()
if  ret == 0:
    print("stopstream success")
else:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret == 0:
    print("close device successful")
else:
    print('scCloseDevice failed: ' + str(ret)) 
           