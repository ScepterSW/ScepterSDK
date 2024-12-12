from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
camera = ScepterTofCam()

print('---ColorExposureTimeSetGet---')
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
	print("connect status:",device_info.status)  
	print("Call scOpenDeviceBySN with connect status :",ScConnectStatus.SC_CONNECTABLE.value)
	exit()
else:
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret == 0:
    print("scOpenDeviceBySN,status :"+ str(ret))
else:
    print('scOpenDeviceBySN failed: ' + str(ret))
    exit()

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed:"+ str(ret))
    exit()

'''Get default frame rate'''
ret, rate = camera.scGetFrameRate()
if ret != 0:
    print("scGetFrameRate failed status:"+ str(ret))
    exit()

'''if need change the framerate, do first'''
print("---- To  SC_EXPOSURE_CONTROL_MODE_MANUAL ----")
'''switch exposure mode to manual'''
ret = camera.scSetExposureControlMode(ScSensorType.SC_COLOR_SENSOR,ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL)

if ret != 0:
    print("scSetExposureControlMode failed status:" + str(ret))
    exit()
else:
    print("scSetExposureControlMode  ok")

print("* step1. Get Color exposure time range with frameRate " + str(rate) + "*")

'''Get the range of the Color exposure time'''
ret, exposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_COLOR_SENSOR)
if ret != 0:
    print("scGetMaxExposureTime failed status:" + str(ret))
    exit()

print("Recommended scope: 100 - " + str(exposureTime))

print("* step2. Set and Get new ExposureTime *" )
'''Set new ExposureTime '''
ret = camera.scSetExposureTime(ScSensorType.SC_COLOR_SENSOR, 3000)
if ret != 0:
    print("scSetExposureTime failed status:" + str(ret))
    exit()
else:
    print("SetExposureTime:3000")


ret, params = camera.scGetExposureTime(ScSensorType.SC_COLOR_SENSOR);
if ret != 0:
    print("scGetExposureTime failed status:" + str(ret))
    exit()
else:
     print("GetExposureTime:" + str(params))
print("* Set and Get ColorGain *")
'''set new ColorGain'''
gain = c_float(3.5)
ret = camera.scSetColorGain(gain)
if  ret == 0:
    print("SetColorGain:" + str(gain.value))
else:
    print("scSetColorGain failed status:"+ str(ret))
    exit()

ret,resut = camera.scGetColorGain()
if  ret == 0:
    print("GetColorGain:" + str(resut[0]))
else:
    print("scGetColorGain failed status:"+ str(ret))
    exit()

print("---- To SC_EXPOSURE_CONTROL_MODE_AUTO ----")
'''switch exposure mode to auto'''
ret = camera.scSetExposureControlMode(ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO)
if 0 != ret:
    print("scSetExposureControlMode failed status:" + str(ret))
    exit()
else:
    print("scSetExposureControlMode ok")

print("* step1. Get Color exposure time range *")
'''Get the range of the Auto Color exposure time'''
ret , exposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_COLOR_SENSOR)
if 0 != ret:
    print("scGetMaxExposureTime failed status:" + str(ret))
    exit()
print("Recommended scope: 100 - " + str(exposureTime))

print("* step2. Set and Get new Auto Max Color exposure time range *" )
'''set new range of Auto Color exposure time.[100 maxExposureTime.exposureTime]'''
ret = camera.scSetColorAECMaxExposureTime(3000)
if 0 != ret:
    print("scSetExposureTimeAutoMax failed status:" + str(ret))
    exit()
else:
    print("SetExposureTimeAutoMax:3000")

'''Get the new range of the Auto Color exposure time. '''
ret, params = camera.scGetColorAECMaxExposureTime();
if 0 != ret:
    print("scGetExposureTimeAutoMax failed status:" + str(ret))
    exit()
else:
    print("GetExposureTimeAutoMax:" + str(params))

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

print('---end---')