from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
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
    print("scOpenDeviceBySN")
else:
    print('scOpenDeviceBySN failed: ' + str(ret))

'''Set the device as non-DHCP mode.'''
ret = camera.scSetDeviceDHCPEnabled(False)
if ret == 0:
    print("Set device DHCP disabled.")
else:
	print("scSetDeviceDHCPEnabled failed status:" + str(ret))
	exit()

'''Set the IP address of the device in non-DHCP mode.'''
strIP = r"192.168.1.102"
c_strIP = (c_char * 16)(*bytes(strIP, 'utf-8'))
ret = camera.scSetDeviceIPAddr(byref(c_strIP), 16)
if ret == 0:
    print("Set IP:" + strIP + " OK.")
else:
	print("Set IP failed status:" + str(ret))
	exit()

'''Set the subnet mask of the device in non-DHCP mode.'''
strSubMask = r"255.255.255.0"
c_strSubMask= (c_char * 16)(*bytes(strSubMask, 'utf-8'))
ret = camera.scSetDeviceSubnetMask(byref(c_strSubMask), 16)
if ret == 0:
    print("Set SubnetMask:" + strSubMask + " OK." )
else:
	print("Set subnetMask failed status:" + str(ret))
	exit()

'''When the device is rebooted, the set IP and subnet mask take effect.'''
ret = camera.scRebootDevie()
if ret != 0:
	print("scRebootDevie failed status:" + str(ret))
	exit()

ret = camera.scCloseDevice()  
if  ret == 0:
    print("scCloseDevice successful")
else:
    print('scCloseDevice failed: ' + str(ret))   
           