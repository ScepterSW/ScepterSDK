from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time

camera = ScepterTofCam()


def HotPlugStateCallback(type_struct,  state = c_int32(0)):
    global camera
    if state ==0:
        print(str(type_struct.contents.serialNumber) + "   add")
        ret = camera.scOpenDeviceBySN(type_struct.contents.serialNumber)
        if  ret == 0:
            print(str(type_struct.contents.serialNumber) + " open success")
        else:
            print(str(type_struct.contents.serialNumber) + " open failed",ret)
        ret = camera.scStartStream()
        if  ret == 0:
            print(str(type_struct.contents.serialNumber) + " startstream success")
        else:
            print(str(type_struct.contents.serialNumber) + " startstream failed",ret)
    else:
        print(str(type_struct.contents.serialNumber) + "   remove")
        ret = camera.scStopStream()
        if  ret == 0:
            print(str(type_struct.contents.serialNumber) + " stopstream success")
        else:
            print(str(type_struct.contents.serialNumber) + " stopstream failed",ret)
        ret = camera.scCloseDevice()
        if  ret == 0:
            print(str(type_struct.contents.serialNumber) + " close success")
        else:
            print(str(type_struct.contents.serialNumber) + " close failed",ret)

camera.scSetHotPlugStatusCallback(HotPlugStateCallback)

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

print("serialNumber: "+str(device_info.serialNumber))
ret = camera.scOpenDeviceBySN(device_info.serialNumber)

if  ret == 0 or ret == -103:
    ret = camera.scStartStream()
    if  ret == 0:
        print("startstream success")
    else:
        print("startstream failed",ret)

    while 1:
        time.sleep(1)
else:
    print('scOpenDeviceBySN failed: ' + str(ret))  

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
                       
