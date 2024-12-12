from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
import psutil

def get_current_pid():
    # 获取当前进程的PID
    current_pid = psutil.Process().pid
    return current_pid

def get_memory_usage(pid):
    # 获取指定PID的内存占用，单位为MB
    process = psutil.Process(pid)
    memory_info = process.memory_info()
    mem_usage = memory_info.rss / (1024 ** 2)  # rss表示已使用的物理内存
    return mem_usage

def get_cpu_usage(pid):
    # 获取指定PID的CPU占用百分比
    process = psutil.Process(pid)
    cpu_percent = process.cpu_percent(interval=0.5)  # 这里interval参数可选，用于统计间隔时间内的CPU使用率
    return cpu_percent


camera = ScepterTofCam()

def displayKeyconfiguration():
    ret, res = camera.scGetSDKVersion()
    if ret != 0:
        print("scGetSDKVersion failed status:", ret)
        return -1
    print("********scGetSDKVersion: ",res," ********")
    ret, res = camera.scGetFirmwareVersion()
    if ret != 0:
        print("scGetFirmwareVersion failed status:", ret)
        return -1
    print("********scGetFirmwareVersion: ",res," ********")
    ret, res = camera.scAIModuleGetEnabled()
    if ret != 0:
        print("scAIModuleGetEnabled failed status:", ret)
        return -1
    print("********scAIModuleGetEnabled: ",res," ********")
    if res == True:
        print("******** Config File ERROR, Please check the config file first ********")
        # ret = camera.scAIModuleSetEnabled(c_bool(False))
        # if ret != 0:
        #     print("scAIModuleGetEnabled failed status: ",ret)
        #     return -1

    ret, res = camera.scAIModuleGetInputFrameTypeEnabled(ScFrameType.SC_COLOR_FRAME)
    if ret != 0:
        print("scAIModuleGetInputFrameTypeEnabled SC_COLOR_FRAME failed status:", ret)
        return -1
    print("********scAIModuleGetInputFrameTypeEnabled SC_COLOR_FRAME : ",res," ********")

    ret, res = camera.scAIModuleGetInputFrameTypeEnabled(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    if ret != 0:
        print("scAIModuleGetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME failed status:", ret)
        return -1
    print("********scAIModuleGetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME: ",res," ********")

    ret, res = camera.scAIModuleGetPreviewFrameTypeEnabled(ScFrameType.SC_COLOR_FRAME)
    if ret != 0:
        print("scAIModuleGetPreviewFrameTypeEnabled SC_COLOR_FRAME failed status:", ret)
        return -1
    print("********scAIModuleGetPreviewFrameTypeEnabled SC_COLOR_FRAME: ",res," ********")

    ret, res = camera.scAIModuleGetPreviewFrameTypeEnabled(ScFrameType.SC_IR_FRAME)
    if ret != 0:
        print("scAIModuleGetPreviewFrameTypeEnabled SC_IR_FRAME failed status:", ret)
        return -1
    print("********scAIModuleGetPreviewFrameTypeEnabled SC_IR_FRAME: ",res," ********")

    ret, res = camera.scAIModuleGetPreviewFrameTypeEnabled(ScFrameType.SC_DEPTH_FRAME)
    if ret != 0:
        print("scAIModuleGetPreviewFrameTypeEnabled SC_DEPTH_FRAME failed status:", ret)
        return -1
    print("********scAIModuleGetPreviewFrameTypeEnabled SC_DEPTH_FRAME: ",res," ********")

    ret, res = camera.scGetFrameRate()
    if ret != 0:
        print("scGetFrameRate failed status:", ret)
        return -1
    else:
        print("********scGetFrameRate : ", res, "********")


    res,pW,pH = camera.scGetColorResolution()
    if ret != 0:
        print("scGetColorResolution failed status:", ret)
        return -1
    else:
        print("********scGetColorResolution pW : ", pW, " pH: ", pH,"********")


print("---CameraModelSigleFrameDelayTest---")
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
        print('GetDeviceListInfo failed status:' + ret)
        exit()  
else: 
    print("there are no camera found")
    exit()

if  ScConnectStatus.SC_CONNECTABLE.value != device_info.status:
	print("connect status:",device_info.status)  
	print("The device state does not support connection.")
	exit()
else:
    print("productName: "+str(device_info.productName))
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret != 0:
    print('OpenDevice failed status:' + str(ret))
    exit()

displayKeyconfiguration()
ret, frameRate = camera.scGetFrameRate()
if ret == 0:
    print("frameRate :", frameRate)
else:
    print("scGetFrameRate :", ret)
    exit()

ret = camera.scSetWorkMode(ScWorkMode.SC_SOFTWARE_TRIGGER_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed status:"+ str(ret))
    exit()

curPath = os.getcwd()
filename = curPath + "/CameraModelSigleFrameDelayTest.csv"
try:
    file = open(filename, "w")
except IOError:
    print('csv file open failed')
    exit()
file.write("frameIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure\n")

'''
//1.software trigger.
//2.ReadNextFrame.
//3.GetFrame acoording to Ready flag and Frametype.
//4.sleep 1000/frameRate (ms)
'''
number = int(input('Please input the number of tests:'))
for i in range(number):
    ret = camera.scSoftwareTriggerOnce()
    startTime = int(time.time()*1000)
    if  ret != 0:  
        print("scSoftwareTriggerOnce failed:",ret)

    ret, frameready = camera.scGetFrameReady(c_uint16(1200))
    if  ret != 0:
        print("scGetFrameReady failed status:",ret)
        continue       
    if  frameready.depth:      
        ret,frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
        endTime = int(time.time()*1000)
        if  ret == 0:
            if 0 == frame.frameIndex % 10:
                print("scGetFrame,status:", ret, "  ", "frameType:", frame.frameType, "  ", "frameIndex:", frame.frameIndex)
            exposureEndTime = frame.hardwaretimestamp
            frameIntervalTime = endTime - startTime
            frameIntervalNtpTime = endTime - exposureEndTime
            current_pid = get_current_pid()
            mem_usage = get_memory_usage(current_pid)
            cpu_usage = get_cpu_usage(current_pid)
            #print(f"当前进程PID: {current_pid}, 内存占用: {mem_usage:.2f} MB,CPU占用: {cpu_usage}%")
            if frameIntervalNtpTime > 2000:
                file.write("{0},{1},{2},{3},N/A\n".format(frame.frameIndex, cpu_usage, mem_usage,frameIntervalTime))
            else:
                file.write("{0},{1},{2},{3},{4}\n".format(frame.frameIndex, cpu_usage, mem_usage, frameIntervalTime, frameIntervalNtpTime))
    time.sleep(1/frameRate)
file.close()

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret != 0:  
    print("scSetWorkMode failed status:",ret)

ret = camera.scStopStream()
if  ret != 0:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret != 0:
    print('scCloseDevice failed: ' + str(ret))
           