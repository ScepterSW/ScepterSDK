from pickle import FALSE, TRUE
import sys
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
print(libpath + "API")
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time

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
    #print(char_array_obj.value)
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
    ret = camera.scAIModuleSetWorkMode(ScAIModuleMode.AI_CONTINUOUS_RUN_MODE)
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



print("---AIContinuousRunMode---")
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

ret = setParameters()
if ret != 0:
    print("setParameters failed status:",ret)
    exit()

ret, frameRate = camera.scGetFrameRate()
if ret == 0:
    print("frameRate :", frameRate)
else:
    print("scGetFrameRate :", ret)
    exit()

ret = camera.scStartStream()
if  ret != 0:
    print("scStartStream failed status:"+ str(ret))
    exit()
#Get the result form AI module.
ResultCount = 100
for i in range(ResultCount):
    ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
    if ret != 0:
        print(" scAIModuleGetResult failed status:", ret)
        time.sleep(0.005)
        continue
    print("AIModuleGetResult resultIndex :", result.resultIndex)

ret = camera.scStopStream()
if  ret != 0:
    print("stopstream failed",ret)

ret = camera.scCloseDevice()     
if  ret != 0:
    print('scCloseDevice failed: ' + str(ret))
    