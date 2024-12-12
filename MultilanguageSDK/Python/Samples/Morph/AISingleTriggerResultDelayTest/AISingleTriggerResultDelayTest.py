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

camera = ScepterTofCam()


def setParameters():
    ret = camera.scSetFrameRate(15)
    if ret != 0:
        print("scSetFrameRate failed: ", ret)
        return ret

    ret = camera.scSetColorResolution(640,480)
    if ret != 0:
        print("scSetColorResolution failed: " + ret)
        return ret

    pbuffer = ctypes.c_void_p()
    pbufferSize = ctypes.c_uint16(0)
    ret = camera.scAIModuleGetParam(paramID = c_uint32(4), pBuffer = pbuffer, bufferSize = pbufferSize)
    if ret != 0:
        print("scAIModuleGetParam failed: " + ret)
        return ret
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("scAIModuleGetParam paramID: 4 status:", ret, " text(ASCII): " , str_buffer)

    strInfo = "Hello world"
    char_array_obj = create_string_buffer(strInfo.encode(), 13)

    ret = camera.scAIModuleSetParam(paramID=c_uint32(4),pBuffer = pointer(char_array_obj), bufferSize = pbufferSize)
    if ret != 0:
        print("scAIModuleSetParam failed: " + ret)
        return ret
    print("scAIModuleSetParam paramID: 4 status:" , ret ," text(ASCII): ", strInfo)

    ret = camera.scAIModuleGetParam(paramID=c_uint32(4), pBuffer = pbuffer, bufferSize = pbufferSize)
    if ret != 0:
        print("scAIModuleGetParam failed: " + ret)
        return ret
    str_buffer = ctypes.string_at(pbuffer, pbufferSize.value).decode('ascii')
    print("scAIModuleGetParam paramID: 4 status:", ret, " text(ASCII): " , str_buffer)

    # Get the parameter with the parameter ID 5.
    ret = camera.scAIModuleGetParam(paramID = c_uint32(5), pBuffer = pbuffer, bufferSize = pbufferSize)
    if ret != 0 or int(pbufferSize.value) < 1:
        print("scAIModuleGetParam failed: " + ret)
        return ret

    pbufferUint = ctypes.string_at(pbuffer, pbufferSize.value)
    print("scAIModuleGetParam paramID: 5 status:" , ret , " text(HEX): ", pbufferUint)
    # Set the parameter with the parameter ID 5.
    buffer_HEX = (c_uint8 * 3)(0x01, 0x02, 0x03 )
    ret = camera.scAIModuleSetParam(paramID=c_uint32(5),pBuffer = pointer(buffer_HEX), bufferSize = pbufferSize)
    if ret != 0:
        print("scAIModuleSetParam failed: " + ret)
        return ret
    res = ""
    for c in range(len(buffer_HEX)):
        res += hex(buffer_HEX[c]) + " "
    print("scAIModuleSetParam paramID: 5 status:" , ret ," text(HEX): ", res)

    ret = camera.scAIModuleGetParam(paramID=c_uint32(5), pBuffer = pbuffer, bufferSize = pbufferSize)
    if ret != 0:
        print("scAIModuleGetParam failed: " + ret)
        return ret
    buffer_HEX_new = ctypes.string_at(pbuffer, pbufferSize.value)
    print("scAIModuleGetParam paramID: 5 status:", ret, " text(HEX): " , buffer_HEX_new)

    # Set input frame type of SC_COLOR_FRAME enabled.
    ret = camera.scAIModuleSetInputFrameTypeEnabled(ScFrameType.SC_COLOR_FRAME, c_bool(True))
    if ret != 0:
        print("scAIModuleSetInputFrameTypeEnabled failed: ", ret)
        return ret


    # Set input frame type of SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME enabled.
    ret = camera.scAIModuleSetInputFrameTypeEnabled(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, c_bool(True))
    if ret != 0:
        print("scAIModuleSetInputFrameTypeEnabled SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME failed: ", ret)
        return ret

	# Set preview frame type of SC_DEPTH_FRAME disabled.
    ret = camera.scAIModuleSetPreviewFrameTypeEnabled(ScFrameType.SC_DEPTH_FRAME, c_bool(False))
    if ret != 0:
        print("scAIModuleSetPreviewFrameTypeEnabled SC_DEPTH_FRAME failed: ", ret)
        return ret

	# Set preview frame type of SC_IR_FRAME disabled.
    ret = camera.scAIModuleSetPreviewFrameTypeEnabled(ScFrameType.SC_IR_FRAME, c_bool(False))
    if ret != 0:
        print("scAIModuleSetPreviewFrameTypeEnabled SC_IR_FRAME failed: ", ret)
        return ret


    # Set AI module continuous running.
    ret = camera.scAIModuleSetWorkMode(ScAIModuleMode.AI_SINGLE_RUN_MODE)
    if ret != 0:
        print("scAIModuleSetWorkMode failed: ", ret)
        return ret

	# Enable AI module.
    ret = camera.scAIModuleSetEnabled(c_bool(True))
    if ret != 0:
        print("scAIModuleSetEnabled failed: ", ret)
        return ret
    print("setParameters done")
    return 0

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
    if res == False:
        print("******** Config File ERROR, Please check the config file first ********")
        # ret = camera.scAIModuleSetEnabled(c_bool(True))
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

print("---AISingleTriggerResultDelayTest---")
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
    print("serialNumber: "+str(device_info.serialNumber))
    print("ip: "+str(device_info.ip))
    print("connectStatus: "+str(device_info.status))

ret = camera.scOpenDeviceBySN(device_info.serialNumber)
if  ret != 0:
    print('OpenDevice failed status:' + str(ret))
    exit()

# ret = setParameters()
# if ret != 0:
#     print("setParameters failed status:",ret)
#     exit()

ret, frameRate = camera.scGetFrameRate()
if ret == 0:
    print("frameRate :", frameRate)
else:
    print("scGetFrameRate :", ret)
    exit()
displayKeyconfiguration()
ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed status:"+ str(ret))
    exit()
curPath = os.getcwd()
filename = curPath + "/AISingleTriggerResultDelayTest.csv"
try:
    file = open(filename, "w")
except IOError:
    print('csv file open failed')
    exit()
file.write("resultIndex,cpu,mem,TotalDelay,ExcludeDelayofExposure\n")
#Get the result form AI module.
ResultCount = int(input("Please input count: "))
for i in range(ResultCount):
    # Call the below api to trigger one result, then the result will be sent.
    # If do not call this function, the result will not be sent and the API of scAIModuleGetResult will return timeout fail.
    ret = camera.scSoftwareTriggerOnce()
    startTime = int(time.time()*1000)
    #print("AIModuleGetResult startTime :", startTime)
    if ret != 0:
        print("scSoftwareTriggerOnce failed status:", ret)
        continue
    ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
    endTime = int(time.time()*1000)
    #print("AIModuleGetResult endTime1 :", endTime)
    if ret != 0:
        print("scAIModuleGetResult failed status:", ret)
        time.sleep(0.005)
        continue
    #print("AIModuleGetResult resultTimestamp1 :", result.resultTimestamp)
    resultPtr = ctypes.cast(result.pResultData, c_char_p)
    resStr = resultPtr.value.decode("utf-8")
    if resStr.find("{") == -1:
        # get once more for the triggered result, need  a while loop here
        print("The last usless res:", result.resultIndex)
        while TRUE:
            ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
            endTime = int(time.time()*1000)
            #print("AIModuleGetResult endTime2 :", endTime)
            #print("AIModuleGetResult resultTimestamp2 :", result.resultTimestamp)
            if ret != 0:
                print("scAIModuleGetResult failed status:", ret)
                time.sleep(0.005)
                continue
            else:
                resultPtr = ctypes.cast(result.pResultData, c_char_p)
                resStr = resultPtr.value.decode("utf-8")
                if resStr.find("{") != -1:
                    print("get real res  ", result.resultIndex)
                    #print("AIModuleGetResult resultTimestamp3 :", result.resultTimestamp)
                    break
        # resultPtr = ctypes.cast(result.pResultData, c_char_p)
        # resStr = resultPtr.value.decode("utf-8")
    #print("scAIModuleGetResult ", resultPtr.value.decode("utf-8"), result.dataLen)
    exposureEndTime = result.resultTimestamp
    frameIntervalTime = endTime - startTime
    frameIntervalNtpTime = endTime - exposureEndTime
    if result.resultIndex % 10 == 0:
        print("AIModuleGetResult resultIndex :", result.resultIndex, "AIModuleGetResult frameIntervalTime :", frameIntervalTime)
    # print("AIModuleGetResult dataLen :", result.dataLen)
    # print("AIModuleGetResult resultTimestamp :", result.resultTimestamp)
    current_pid = get_current_pid()
    mem_usage = get_memory_usage(current_pid)
    cpu_usage = get_cpu_usage(current_pid) # block interface, need to adapt with scAIModuleGetResult timeout, now the algorithm defaults timeout is 1 second
    #print(f"当前进程PID: {current_pid}, 内存占用: {mem_usage:.2f} MB,CPU占用: {cpu_usage}%")
    if frameIntervalNtpTime > 2000:
        file.write("{0},{1},{2},{3},N/A\n".format(result.resultIndex, cpu_usage, mem_usage,frameIntervalTime))
    else:
        file.write("{0},{1},{2},{3},{4}\n".format(result.resultIndex, cpu_usage, mem_usage, frameIntervalTime, frameIntervalNtpTime))

ret = camera.scStopStream()
if  ret != 0:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret != 0:
    print('scCloseDevice failed: ' + str(ret))
    