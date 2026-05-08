from pickle import FALSE, TRUE
import sys
currentPath = sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path
from API.ScepterDS_api import *
import time
print('---ColorExposureTimeSetGet---')
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

'''Get default frame rate'''
ret, rate = camera.scGetFrameRate()
if  ret == 0:
    print('[scGetFrameRate] success ScStatus({}). The device frame rate is {}.\n'.format(str(ret), rate))
else:
    print('[scGetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

'''if need change the framerate, do first'''
print("---- To  SC_EXPOSURE_CONTROL_MODE_MANUAL ----")
'''switch exposure mode to manual'''
ret = camera.scSetExposureControlMode(ScSensorType.SC_COLOR_SENSOR,ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL)

if ret == 0:
    print('[scSetExposureControlMode] success ScStatus({}). Set SC_EXPOSURE_CONTROL_MODE_MANUAL success.'.format(str(ret)))
else:
    print('[scSetExposureControlMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("--- 1. Get color sensor exposure time range with frame rate " + str(rate) + " ---")

'''Get the range of the Color exposure time'''
ret, exposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_COLOR_SENSOR)
if ret == 0:
    print('[scGetMaxExposureTime] success ScStatus({}). Recommended scope: 100 - {}'.format(str(ret) ,str(exposureTime)))
else:
    print('[scGetMaxExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("--- 2. Set and Get new ExposureTime ---" )
'''Set new ExposureTime '''
ret = camera.scSetExposureTime(ScSensorType.SC_COLOR_SENSOR, 3000)
if ret == 0:
    print('[scSetExposureTime] success ScStatus({}). Set the device exposure time to 3000'.format(str(ret)))
else:
    print('[scSetExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret, params = camera.scGetExposureTime(ScSensorType.SC_COLOR_SENSOR)
if ret == 0:
    print('[scGetExposureTime] success ScStatus({}).The device exposure time is {}'.format(str(ret) ,str(params)))
else:
    print('[scGetExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)

colorGain = 3.5
ret = camera.scSetColorGain(c_float(colorGain))
if 0 == ret:
    print('[scSetColorGain] success ScStatus({}). Set the device color gain to  {}'.format(str(ret) ,str(colorGain)))
else:
    print('[scSetColorGain] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret, colorGainVal = camera.scGetColorGain()
if 0 == ret:
    print('[scGetColorGain] success ScStatus({}). The device color gain is {} \n'.format(str(ret) ,str(colorGainVal)))
else:
    print('[scGetColorGain] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("---- To SC_EXPOSURE_CONTROL_MODE_AUTO ----")
'''switch exposure mode to auto'''
ret = camera.scSetExposureControlMode(ScSensorType.SC_COLOR_SENSOR, ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_AUTO)
if 0 == ret:
    print('[scSetExposureControlMode] success ScStatus({}). Set SC_EXPOSURE_CONTROL_MODE_AUTO success.'.format(str(ret)))
else:
    print('[scSetExposureControlMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("--- 1. Get Color exposure time range ---")
'''Get the range of the Auto Color exposure time'''
ret , exposureTime = camera.scGetMaxExposureTime(ScSensorType.SC_COLOR_SENSOR)
if 0 == ret:
    print('[scGetMaxExposureTime] success ScStatus({}). Recommended scope: 100 - {}'.format(str(ret) ,str(exposureTime)))
else:
    print('[scGetMaxExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)

print("--- 2. Set and Get new Auto Max Color exposure time range ---" )
'''set new range of Auto Color exposure time.[100 maxExposureTime.exposureTime]'''
ret = camera.scSetColorAECMaxExposureTime(3000)
if 0 == ret:
    print('[scSetColorAECMaxExposureTime] success ScStatus({}). Set color AEC max exposure time to 3000.'.format(str(ret)))
else:
    print('[scSetColorAECMaxExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)

'''Get the new range of the Auto Color exposure time. '''
ret, params = camera.scGetColorAECMaxExposureTime()
if 0 == ret:
    print('[scGetColorAECMaxExposureTime] success ScStatus({}). Get color AEC max exposure time is {}.\n'.format(str(ret), str(params)))
else:
    print('[scGetColorAECMaxExposureTime] fail ScStatus({}).'.format(str(ret)))
    exit(1)
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