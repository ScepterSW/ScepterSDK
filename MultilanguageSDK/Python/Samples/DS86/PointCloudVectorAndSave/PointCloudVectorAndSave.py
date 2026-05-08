from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
from ctypes import POINTER, c_uint8, cast, memmove
print("---PointCloudVectorAndSave---")
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

ret, cameraParam = camera.scGetSensorIntrinsicParameters(ScSensorType.SC_TOF_SENSOR)
if  ret == 0:
    print('[scGetSensorIntrinsicParameters] success ScStatus({}).'.format(str(ret)))
else:
    print('[scGetSensorIntrinsicParameters] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)

#Wait for the device to upload image data.
time.sleep(1)

for ind in range(10):
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
            curPath = os.getcwd()
            file = open(curPath + "/PointCloud.txt","w")

            WINDOW_SIZE = 100
            frameSize16 = frame.width*frame.height
            pDepthFrameData = (c_uint16 * frameSize16)()
            memmove(pDepthFrameData, frame.pFrameData, frame.width*frame.height*2)

            offset = (frame.height - WINDOW_SIZE) /2 * frame.width
            for i in range(int((frame.height - WINDOW_SIZE) /2), int((frame.height + WINDOW_SIZE)/2)):
                for j in range(int((frame.width - WINDOW_SIZE) /2), int((frame.width + WINDOW_SIZE)/2)):
                    depthPoint = ScDepthVector3()
                    depthPoint.depthX = j
                    depthPoint.depthY = i
                    depthPoint.depthZ = pDepthFrameData[int(offset + j)]
                    ret, worldV = camera.scConvertDepthToPointCloud(depthPoint, 1, cameraParam)
                    if ret == 0:
                        #print('[scConvertDepthToPointCloud] success ScStatus({}).'.format(str(ret)))
                        if 0 < worldV[0].z and worldV[0].z < 0xFFFF:
                            file.write("{0}\t{1}\t{2}\n".format(worldV[0].x, worldV[0].y, worldV[0].z))
                    else:
                        print('[scConvertDepthToPointCloud] fail ScStatus({}).'.format(str(ret)))
                offset += frame.width
            print('Save point cloud successful in PointCloud.txt')
            file.close()
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