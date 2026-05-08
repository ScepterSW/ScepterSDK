from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
print(libpath + "API")
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
print("---AISingleTriggerResultDelayTest---")
camera = ScepterTofCam()

def setParameters():
    # Set AI module continuous running.
    ret = camera.scAIModuleSetWorkMode(ScAIModuleMode.AI_SINGLE_RUN_MODE)
    if ret == 0:
        print("[scAIModuleSetWorkMode] success, ScStatus({}). Set AI_SINGLE_RUN_MODE.".format(ret))
    else:
        print("[scAIModuleSetWorkMode] fail, ScStatus({})".format(ret))
        return ret

	# Enable AI module.
    ret = camera.scAIModuleSetEnabled(c_bool(True))
    if ret == 0:
        print("[scAIModuleSetEnabled] success, ScStatus({}). Enable AI module.".format(ret))
    else:
        print("[scAIModuleSetEnabled] fail, ScStatus({})".format(ret))
        return ret
    return 0

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

ret = setParameters()
if  ret == 0:
    print('[setParameters] success ScStatus({}).'.format(str(ret)))
else:
    print('[setParameters] fail ScStatus({}).'.format(str(ret)))
    exit(1)

pbuffer = ctypes.c_void_p()
pbufferSize = ctypes.c_uint16(0)
ret = camera.scAIModuleGetParam(paramID = c_uint32(0), pBuffer = pbuffer, bufferSize = pbufferSize)
if ret == 0:
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("[scAIModuleGetParam] success ScStatus({}). The algo's version is: {}".format(str(ret), str(str_buffer)))
else:
    print('[scAIModuleGetParam] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scAIModuleGetParam(paramID = c_uint32(1), pBuffer = pbuffer, bufferSize = pbufferSize)
if ret == 0:
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("[scAIModuleGetParam] success ScStatus({}). The algo's name is: {}".format(str(ret), str(str_buffer)))
else:
    print('[scAIModuleGetParam] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret, frameRate = camera.scGetFrameRate()
if  ret == 0:
    print('[scGetFrameRate] success ScStatus({}). The device frame rate is {}.'.format(str(ret), frameRate))
else:
    print('[scGetFrameRate] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scSetWorkMode(ScWorkMode.SC_SOFTWARE_TRIGGER_MODE)
if  ret == 0:
    print('[scSetWorkMode] success ScStatus({}). Set SC_SOFTWARE_TRIGGER_MODE.'.format(str(ret)))
else:
    print('[scSetWorkMode] fail ScStatus({}).'.format(str(ret)))
    exit(1)

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1)
curPath = os.getcwd()
filename = curPath + "/AISingleTriggerResultDelayTest.csv"
try:
    file = open(filename, "w")
except IOError:
    print('csv file open failed')
    exit(1)
file.write("resultIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure\n")
#Get the result form AI module.
ResultCount = 100
for i in range(ResultCount):
    # Call the below api to trigger one result, then the result will be sent.
    # If do not call this function, the result will not be sent and the API of scAIModuleGetResult will return timeout fail.
    ret = camera.scSoftwareTriggerOnce()
    startTime = int(time.time()*1000)
    #print("AIModuleGetResult startTime :", startTime)
    if ret != 0:
        print('[scSoftwareTriggerOnce] fail ScStatus({}).'.format(str(ret)))
        continue
    ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
    endTime = int(time.time()*1000)
    #print("AIModuleGetResult endTime1 :", endTime)
    if ret != 0:
        print('[scAIModuleGetResult] fail ScStatus({}).'.format(str(ret)))
        time.sleep(0.005)
        continue
    exposureEndTime = result.resultTimestamp
    frameIntervalTime = endTime - startTime
    frameIntervalNtpTime = endTime - exposureEndTime
    if result.resultIndex % 10 == 0:
        print('[scAIModuleGetResult] success ScStatus({}). ScAIResult <resultIndex>: {}'.format(str(ret), str(result.resultIndex)))
    current_pid = get_current_pid()
    mem_usage = get_memory_usage(current_pid)
    cpu_usage = get_cpu_usage(current_pid) # block interface, need to adapt with scAIModuleGetResult timeout, now the algorithm defaults timeout is 1 second
    #print(f"当前进程PID: {current_pid}, 内存占用: {mem_usage:.2f} MB,CPU占用: {cpu_usage}%")
    if frameIntervalNtpTime > 2000:
        file.write("{0},{1},{2},{3},N/A\n".format(result.resultIndex, cpu_usage, mem_usage,frameIntervalTime))
    else:
        file.write("{0},{1},{2},{3},{4}\n".format(result.resultIndex, cpu_usage, mem_usage, frameIntervalTime, frameIntervalNtpTime))
    time.sleep(1.2/frameRate)

ret = camera.scSetWorkMode(ScWorkMode.SC_ACTIVE_MODE)
if  ret == 0:
    print('[scSetWorkMode] success ScStatus({}). Set SC_ACTIVE_MODE'.format(str(ret)))
else:
    print('[scSetWorkMode] fail ScStatus({}).'.format(str(ret)))
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