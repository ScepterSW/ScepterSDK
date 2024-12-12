from pickle import FALSE, TRUE
import sys

currentPath = sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath)  # absolutely path

from API.ScepterDS_api import *
import time

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
if ret == 0:
    print("scOpenDeviceBySN")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))

imgpath = input('Please input firmware file path:')
ret = camera.scStartUpgradeFirmWare(imgpath)
if ret == 0:
    print("scStartUpgradeFirmWare successful")
else:
    print('scStartUpgradeFirmWare failed: ' + str(ret))

while True:
    ret, status, process = camera.scGetUpgradeStatus()
    if ret == 0:
        print("Upgrade firmWare status:" + str(status) + ", process:" + str(process))
        if status == 0:
            '''Upgrade progress is 100, upgrade successful. After the upgrade is successful,'''
            '''the SDK will automatically reboot the device internally to make the upgrade file effective.'''
            if process == 100:
                print('Upgrade OK.')
                break
        else:
            print('scGetUpgradeStatus failed: ' + str(status))
            break

    else:
        print('scGetUpgradeStatus failed: ' + str(ret))
        break
    time.sleep(1)

ret = camera.scCloseDevice()
if ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret))
