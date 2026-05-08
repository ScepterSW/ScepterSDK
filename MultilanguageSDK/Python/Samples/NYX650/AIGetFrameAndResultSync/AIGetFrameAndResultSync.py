from pickle import FALSE, TRUE
import sys
import threading
from queue import Queue
currentPath =  sys.path[0]
pos = currentPath.find('Samples')
libpath = currentPath[:pos]
print(libpath + "API")
sys.path.append(libpath) #absolutely path

from API.ScepterDS_api import *
import time
print("---AIGetFrameAndResultSync---")
camera = ScepterTofCam()

frameMutex = threading.Lock()
resultMutex = threading.Lock()
QueueMaxNum = 6
frameQueue = []
resultQueue = []
# frameQueue = Queue(QueueMaxNum) # dose not support front method
# resultQueue = Queue(QueueMaxNum)


class DealThread(threading.Thread):
    def __init__(self, type):
        super(DealThread, self).__init__()
        self.type = type
        self.running_ = True
    def run(self):
        if self.type == 1:
            #print("Frame Thread Running")
            while self.running_ :
                depthFrame = ScFrame()
                frameReady = ScFrameReady()
                ret, frameReady = camera.scGetFrameReady(c_uint16(1200))
                if  ret != 0:
                    print("[scGetFrameReady] fail, ScStatus({})".format(ret))
                    continue       
                if  frameReady.depth:
                    # must enable depthFrame before
                    ret, depthFrame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
                    if  ret == 0:
                        frameMutex.acquire()
                        if  len(frameQueue) > QueueMaxNum:
                            frameQueue.pop(0)
                            frameQueue.append(depthFrame)
                        else:
                            frameQueue.append(depthFrame)
                        frameMutex.release()
                    else:
                        print("[scGetFrame] fail, ScStatus({})".format(ret))
        else:
            #print("AI Result Thread Running")
            while self.running_ :
                ret, result = camera.scAIModuleGetResult(waitTime = c_uint32(1200))
                if  ret != 0:
                    print("[scAIModuleGetResult] fail, ScStatus({})".format(ret))
                    continue
                resultMutex.acquire()
                if len(resultQueue) > QueueMaxNum:
                    resultQueue.pop(0)
                    resultQueue.append(result)
                else:
                    resultQueue.append(result)
                resultMutex.release()

def setParameters():
    # Set AI module continuous running.
    ret = camera.scAIModuleSetWorkMode(ScAIModuleMode.AI_CONTINUOUS_RUN_MODE)
    if ret == 0:
        print("[scAIModuleSetWorkMode] success, ScStatus({}). Set AI_CONTINUOUS_RUN_MODE.".format(ret))
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

ret = camera.scStartStream()
if  ret == 0:
    print('[scStartStream] success ScStatus({}).'.format(str(ret)))
else:
    print('[scStartStream] fail ScStatus({}).'.format(str(ret)))
    exit(1) 


# sync
imageThread = DealThread(1)
imageThread.start()
resThread = DealThread(2)
resThread.start()

syncCount = 100
while syncCount > 0:
    frameMutex.acquire()
    resultMutex.acquire()
    while len(frameQueue) > 0 and len(resultQueue) > 0:
        frame = frameQueue[0]
        result = resultQueue[0]
        if frame.hardwaretimestamp == result.resultTimestamp:
            print("Sync frame index:", frame.frameIndex ,", result index: " , result.resultIndex , ", timestamp: " , result.resultTimestamp)
            frameQueue.pop(0)
            resultQueue.pop(0)
            syncCount = syncCount - 1
        elif frame.hardwaretimestamp > result.resultTimestamp:
            print("Sync result not match bigger, resultIndex: " , result.resultIndex , ", timestamp: " , result.resultTimestamp , ", frameIndex:" , frame.frameIndex , ", timestamp: " , frame.hardwaretimestamp)
            resultQueue.pop(0)
        else:
            print("Sync result not match less , resultIndex: " , result.resultIndex , ", timestamp: " , result.resultTimestamp , ", frameIndex:" , frame.frameIndex , ", timestamp: " , frame.hardwaretimestamp)
            frameQueue.pop(0)
    resultMutex.release()
    frameMutex.release()
    time.sleep(0.01)
imageThread.running_ = False
resThread.running_ = False
imageThread.join()
resThread.join()

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