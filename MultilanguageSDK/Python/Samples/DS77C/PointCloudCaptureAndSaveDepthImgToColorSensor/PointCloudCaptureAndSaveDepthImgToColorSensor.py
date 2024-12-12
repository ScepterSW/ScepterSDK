from pickle import FALSE, TRUE
import sys

currentPath = sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath)  # absolutely path

from API.ScepterDS_api import *
import time
from ctypes import POINTER, c_uint8, cast, memmove

camera = ScepterTofCam()

camera_count = camera.scGetDeviceCount(3000)
print("Get device count:", camera_count)
if camera_count <= 0:
    print(
        "scGetDeviceCount scans for 3000ms and then returns the device count is 0. Make sure the device is on the network before running the samples.")
    exit()

device_info = ScDeviceInfo()

if camera_count > 0:
    ret, device_infolist = camera.scGetDeviceInfoList(camera_count)
    if ret == 0:
        device_info = device_infolist[0]
    else:
        print(' failed:' + ret)
        exit()
else:
    print("there are no camera found")
    exit()

if ScConnectStatus.SC_CONNECTABLE.value != device_info.status:
    print("connect status:", device_info.status)
    print("Call scOpenDeviceBySN with connect status :", ScConnectStatus.SC_CONNECTABLE.value)
    exit()
else:
    print("serialNumber: " + str(device_info.serialNumber))
    print("ip: " + str(device_info.ip))
    print("connectStatus: " + str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if ret != 0:
    print('scOpenDeviceBySN failed: ' + str(ret))
    exit()

print("scOpenDeviceBySN,status :" + str(ret))

ret = camera.scSetColorResolution(800, 600)
if ret != 0:
    print("scSetColorResolution failed:" + str(ret))
    exit()

ret = camera.scStartStream()
if ret != 0:
    print("scStartStream failed:" + str(ret))
    exit()

ret = camera.scSetTransformDepthImgToColorSensorEnabled(True)
if ret != 0:
    print("scSetTransformDepthImgToColorSensorEnabled failed:" + str(ret))
    exit()

while 1:
    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if ret != 0:
        print("scGetFrameReady failed status:", ret)
        continue

    if frameready.transformedDepth:
        ret, frame = camera.scGetFrame(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
        if ret == 0:
            curPath = os.getcwd()
            folder = curPath + "/save"
            if not os.path.exists(folder):
                os.makedirs(folder)
            filename = folder + "/PointCloud.txt"
            file = open(filename, "w")

            frameSize16 = frame.width * frame.height
            ret, worldV = camera.scConvertDepthFrameToPointCloudVector(frame)
            if ret == 0:
                for i in range(int(frameSize16)):
                    if 0 < worldV[i].z and worldV[i].z < 0xFFFF:
                        file.write("{0}\t{1}\t{2}\n".format(worldV[i].x, worldV[i].y, worldV[i].z))
            else:
                print("scConvertDepthFrameToPointCloudVector failed" + str(ret))
            print('Save point cloud successful in PointCloud.txt')
            file.close()
            break

ret = camera.scStopStream()
if ret == 0:
    print("scStopStream success")
else:
    print("scStopStream failed" + str(ret))

ret = camera.scCloseDevice()
if ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret))
