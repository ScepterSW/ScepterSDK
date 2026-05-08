from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---DeviceIPAndSubnetMaskSet---")
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

'''Set the device as non-DHCP mode.'''
ret = camera.scSetDeviceDHCPEnabled(False)
if ret == 0:
    print('[scSetDeviceDHCPEnabled] success ScStatus({}). The device DHCP is close.'.format(str(ret)))
else:
	print('[scSetDeviceDHCPEnabled] fail ScStatus({}).'.format(str(ret)))
	exit(1)

'''Set the IP address of the device in non-DHCP mode.'''
strIP = r"192.168.1.102"
c_strIP = (c_char * 16)(*bytes(strIP, 'utf-8'))
ret = camera.scSetDeviceIPAddr(byref(c_strIP), 16)
if ret == 0:
    print('[scSetDeviceIPAddr] success ScStatus({}). Set the device IP to {}.'.format(str(ret), strIP))
else:
	print('[scSetDeviceIPAddr] fail ScStatus({}).'.format(str(ret)))
	exit(1)

'''Set the subnet mask of the device in non-DHCP mode.'''
strSubMask = r"255.255.255.0"
c_strSubMask= (c_char * 16)(*bytes(strSubMask, 'utf-8'))
ret = camera.scSetDeviceSubnetMask(byref(c_strSubMask), 16)
if ret == 0:
    print('[scSetDeviceSubnetMask] success ScStatus({}). Set the subnet mask to {}.'.format(str(ret), strSubMask))
else:
	print('[scSetDeviceSubnetMask] fail ScStatus({}).'.format(str(ret)))
	exit(1)

'''When the device is rebooted, the set IP and subnet mask take effect.'''
ret = camera.scRebootDevie()
if ret == 0:
    print('[scRebootDevie] success ScStatus({}).'.format(str(ret)))
else:
	print('[scRebootDevie] fail ScStatus({}).'.format(str(ret)))
	exit(1)

print("Waiting for reboot.")
time.sleep(10)
print("Rebort done.")

ret = camera.scCloseDevice()
if  ret == 0:
    print('[scCloseDevice] success ScStatus({}).'.format(str(ret)))
else:
    print('[scCloseDevice] fail ScStatus({}).'.format(str(ret)))
    exit(1)

exit(0)