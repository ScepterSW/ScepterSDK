from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time

frameSpace = 20
print("---TransformColorImgToDepthSensorFrame---")
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

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)
# set Mapper
ret = camera.scSetTransformColorImgToDepthSensorEnabled(c_bool(True))
if  ret == 0:
    print('[scSetTransformColorImgToDepthSensorEnabled] success ScStatus({}).'.format(str(ret)))
else:
    print('[scSetTransformColorImgToDepthSensorEnabled] fail ScStatus({}).'.format(str(ret)))

#Wait for the device to upload image data.
time.sleep(1)

saveonce = False
for i in range(frameSpace):
    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if  ret == 0:
        print('[scGetFrameReady] success ScStatus({}).'.format(str(ret)))
    else:
        print('[scGetFrameReady] fail ScStatus({}).'.format(str(ret)))
        continue
    
    if  frameready.transformedColor:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME)
        if  ret == 0:
            print('[scGetFrame] success ScStatus({}). SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME <frameIndex>: {}'.format(str(ret), str(frame.frameIndex)))
            if saveonce == False:
                curPath = os.getcwd()
                filename = "TransformColorImgToDepthSensorFrame_{}.bin".format(str(frame.frameIndex))
                file = open(curPath + "//" +filename,"wb+")
                for i in range(frame.dataLen):
                    file.write(c_uint8(frame.pFrameData[i]))
                file.close()
                print("Save TransformColorImgToDepthSensorFrame successful in {}".format(filename))
                saveonce = True
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