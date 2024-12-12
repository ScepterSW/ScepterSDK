from API.ScepterDS_define import * 
import platform
import os
import ctypes
import sys
from ctypes import *
gCallbackFuncList=[]

class ScepterTofCam():
    device_handle = c_void_p(0)
    def __init__(self):
        system_ = platform.system().lower()
        machine_ = platform.machine().lower()
        architecture_ = platform.architecture()[0]
        print('system:',system_)
        print('machine_:',machine_)
        print('architecture:',architecture_)
        currentPath = sys.path[0]
        pos = currentPath.find('MultilanguageSDK')
        if system_ == 'linux':
            if machine_ == 'x86_64':
                os_info = os.uname()
                print('os_info:',type(os_info))
                system_info = os_info.version
                print('version:',system_info)
                if system_info.find('16.04') != -1:
                    libpath = os.path.abspath(currentPath[:pos] + "BaseSDK/Ubuntu16.04/Lib/libScepter_api.so")
                    #libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Ubuntu16.04/Lib/libScepter_api.so"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = os.path.abspath(currentPath[:pos] + "BaseSDK/Ubuntu/Lib/libScepter_api.so")
                    #libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Ubuntu/Lib/libScepter_api.so"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
            elif machine_ == 'aarch64':
                libpath = os.path.abspath(currentPath[:pos] + "BaseSDK/AArch64/Lib/libScepter_api.so")
                #libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/AArch64/Lib/libScepter_api.so"
                print(libpath)
                self.sc_cam_lib = cdll.LoadLibrary(libpath)
            else:
                print('do not supported OS', system_, machine_)
                exit()
        elif platform.system() == 'Windows':
            if machine_ == 'amd64':
                if architecture_ == '64bit':
                    libpath = os.path.abspath(currentPath[:pos] + "BaseSDK/Windows/Bin/x64/Scepter_api.dll")
                    #libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Windows/Bin/x64/Scepter_api.dll"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = os.path.abspath(currentPath[:pos] + "BaseSDK/Windows/Bin/x86/Scepter_api.dll")
                    #libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Windows/Bin/x86/Scepter_api.dll"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
            else:
                print('do not supported OS', system_, machine_)
                exit()
        else:
            print('do not supported OS', system_, machine_)
            exit()
            
        self.device_handle = c_void_p(0)
        self.sc_cam_lib.scInitialize()
        
    def __del__(self):
        self.sc_cam_lib.scShutdown()

    def scInitialize(self):
        return self.sc_cam_lib.scInitialize()

    def scShutdown(self):
        return self.sc_cam_lib.scShutdown()

    def scGetSDKVersion(self): 
        tmp = c_char * 64
        version = tmp()
        return self.sc_cam_lib.scGetSDKVersion(version, 63),version.value

    def scGetDeviceCount(self, scanTime = c_uint32(33)):
        count = c_int()
        self.sc_cam_lib.scGetDeviceCount(byref(count), scanTime)
        return count.value
    
    def scGetDeviceInfoList(self, cam_count = 1):
        tmp  = ScDeviceInfo* cam_count
        device_infolist = tmp() 
        return self.sc_cam_lib.scGetDeviceInfoList(cam_count, device_infolist),device_infolist

    def scOpenDeviceBySN(self,  SN=c_char_p()):
        if SN:
            return self.sc_cam_lib.scOpenDeviceBySN(SN, byref(self.device_handle))
        else:
            return ScReturnStatus.SC_INPUT_POINTER_IS_NULL
    
    def scOpenDeviceByIP(self,  ip=c_char_p()):
        if ip:
            return self.sc_cam_lib.scOpenDeviceByIP(ip, byref(self.device_handle))
        else:
            return ScReturnStatus.SC_INPUT_POINTER_IS_NULL, 0
        
    def scCloseDevice(self):
        return self.sc_cam_lib.scCloseDevice(byref(self.device_handle))

    def scStartStream(self):
        return self.sc_cam_lib.scStartStream(self.device_handle)

    def scStopStream(self):
        return self.sc_cam_lib.scStopStream(self.device_handle)

    def scGetFrameReady(self,waitTime = c_uint16(33)):
        frameready = ScFrameReady()
        if not self.device_handle:
            return -3, frameready
        return self.sc_cam_lib.scGetFrameReady(self.device_handle, waitTime, byref(frameready)), frameready

    def scGetFrame(self,  frametype = ScFrameType.SC_DEPTH_FRAME):   
        Scframe = ScFrame() 
        return self.sc_cam_lib.scGetFrame(self.device_handle, frametype.value, byref(Scframe)), Scframe

    def scGetDepthRangeValue(self):
        minValue = c_int16(0)
        maxValue = c_int16(0)
        return self.sc_cam_lib.scGetDepthRangeValue(self.device_handle, byref(minValue), byref(maxValue)), minValue.value, maxValue.value
    
    def scGetSensorIntrinsicParameters(self, sensorType = ScSensorType.SC_TOF_SENSOR):
        CameraParameters = ScSensorIntrinsicParameters()
        return self.sc_cam_lib.scGetSensorIntrinsicParameters(self.device_handle, sensorType.value, byref(CameraParameters)), CameraParameters

    def scGetSensorExtrinsicParameters(self):
        CameraExtrinsicParameters = ScSensorExtrinsicParameters()
        return self.sc_cam_lib.scGetSensorExtrinsicParameters(self.device_handle, byref(CameraExtrinsicParameters)), CameraExtrinsicParameters
    
    def scGetFirmwareVersion(self): 
        tmp = c_char * 64
        fw = tmp()
        return self.sc_cam_lib.scGetFirmwareVersion(self.device_handle, fw, 63),fw.value

    def scGetDeviceMACAddress(self):
        tmp = c_char * 18
        mac = tmp()
        return self.sc_cam_lib.scGetDeviceMACAddress(self.device_handle, mac), mac.value

    def scSetDeviceDHCPEnabled(self, enable = c_bool(True)):
        return self.sc_cam_lib.scSetDeviceDHCPEnabled(self.device_handle, enable)

    def scGetDeviceDHCPEnabled(self):
        enable = c_bool(True)
        return self.sc_cam_lib.scGetDeviceDHCPEnabled(self.device_handle, byref(enable)), enable.value

    def scSetDeviceIPAddr(self, IPAddr=c_char_p(), length=c_int32(16)):
        return self.sc_cam_lib.scSetDeviceIPAddr(self.device_handle, IPAddr, length)

    def scGetDeviceIPAddr(self):
        tmp = c_char * 16
        IPAddr = tmp()
        return self.sc_cam_lib.scGetDeviceIPAddr(self.device_handle, IPAddr), IPAddr.value

    def scSetDeviceSubnetMask(self, subnetMask=c_char_p(), length=c_int32(16)):
        return self.sc_cam_lib.scSetDeviceSubnetMask(self.device_handle, subnetMask, length)

    def scGetDeviceSubnetMask(self):
        tmp = c_char * 16
        subnetMask = tmp()
        return self.sc_cam_lib.scGetDeviceSubnetMask(self.device_handle, subnetMask), subnetMask.value

    def scSetRealTimeSyncConfig(self, params = ScTimeSyncConfig()):
        return self.sc_cam_lib.scSetRealTimeSyncConfig(self.device_handle, params)

    def scGetRealTimeSyncConfig(self):
        pParams = ScTimeSyncConfig()
        return self.sc_cam_lib.scGetRealTimeSyncConfig(self.device_handle, byref(pParams)),pParams

    def scSetFrameRate(self, value = c_uint8(30)):
        return self.sc_cam_lib.scSetFrameRate(self.device_handle, value) 

    def scGetFrameRate(self):
        rate = c_uint8(1)
        return self.sc_cam_lib.scGetFrameRate(self.device_handle, byref(rate)), rate.value

    def scSetWorkMode(self,  mode = ScWorkMode.SC_ACTIVE_MODE):
        return self.sc_cam_lib.scSetWorkMode(self.device_handle, mode.value) 

    def scGetWorkMode(self):
        mode = c_uint8(0)
        return self.sc_cam_lib.scGetWorkMode(self.device_handle, byref(mode)), mode.value

    def scSetSoftwareTriggerParameter(self, param = c_uint8(0)):
        return self.sc_cam_lib.scSetSoftwareTriggerParameter(self.device_handle, param)

    def scGetSoftwareTriggerParameter(self):
        param = c_uint8(0)
        return self.sc_cam_lib.scGetSoftwareTriggerParameter(self.device_handle, byref(param)), param.value

    def scSoftwareTriggerOnce(self):
        return self.sc_cam_lib.scSoftwareTriggerOnce(self.device_handle)
   
    def scSetInputSignalParamsForHWTrigger(self, params = ScInputSignalParamsForHWTrigger()):
        return self.sc_cam_lib.scSetInputSignalParamsForHWTrigger(self.device_handle, params)

    def scGetInputSignalParamsForHWTrigger(self):
        params = ScInputSignalParamsForHWTrigger()
        return self.sc_cam_lib.scGetInputSignalParamsForHWTrigger(self.device_handle, byref(params)), params

    def scSetIRGMMGain(self, gmmgain = c_uint8(20)):
        return self.sc_cam_lib.scSetIRGMMGain(self.device_handle, gmmgain) 

    def scGetIRGMMGain(self):
        gmmgain = c_uint8(1)
        return self.sc_cam_lib.scGetIRGMMGain(self.device_handle, byref(gmmgain)), gmmgain.value

    def scSetIRGMMCorrection(self, params = ScIRGMMCorrectionParams()):
        return self.sc_cam_lib.scSetIRGMMCorrection(self.device_handle, params)

    def scGetIRGMMCorrection(self):
        params = ScIRGMMCorrectionParams()
        return self.sc_cam_lib.scGetIRGMMCorrection(self.device_handle, byref(params)), params

    def scSetColorPixelFormat(self,pixelFormat=ScPixelFormat.SC_PIXEL_FORMAT_BGR_888_JPEG):
        return self.sc_cam_lib.scSetColorPixelFormat(self.device_handle, pixelFormat.value)

    def scSetColorGain(self, params = c_float(1.0)):
        return self.sc_cam_lib.scSetColorGain(self.device_handle,  params)

    def scGetColorGain(self):
        tmp = c_float*1
        params = tmp()
        return self.sc_cam_lib.scGetColorGain(self.device_handle,  params), params

    def scGetSupportedResolutionList(self, type = ScSensorType.SC_TOF_SENSOR, cam_count=1):
        tmp = ScResolutionList * cam_count
        pList = tmp()
        return self.sc_cam_lib.scGetSupportedResolutionList(self.device_handle, type.value, byref(pList)), pList

    def scSetColorResolution(self, w = c_int32(1600), h = c_int32(1200)):
        return self.sc_cam_lib.scSetColorResolution(self.device_handle, w, h)

    def scGetColorResolution(self):
        w = c_int32(1600)
        h = c_int32(1200)
        return self.sc_cam_lib.scGetColorResolution(self.device_handle, byref(w), byref(h)), w.value, h.value

    def scSetExposureControlMode(self, sensorType = ScSensorType.SC_TOF_SENSOR, mode = ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL):
        return self.sc_cam_lib.scSetExposureControlMode(self.device_handle, sensorType.value, mode.value) 

    def scGetExposureControlMode(self, sensorType = ScSensorType.SC_TOF_SENSOR):
        mode = c_uint8(1)
        return self.sc_cam_lib.scGetExposureControlMode(self.device_handle, sensorType.value, byref(mode)), mode.value

    def scSetExposureTime(self, sensorType = ScSensorType.SC_TOF_SENSOR, params = c_int32(0)):
        return self.sc_cam_lib.scSetExposureTime(self.device_handle, sensorType.value, params)

    def scGetExposureTime(self, sensorType = ScSensorType.SC_TOF_SENSOR):
        params = c_int32(0)
        return self.sc_cam_lib.scGetExposureTime(self.device_handle, sensorType.value, byref(params)), params.value

    def scSetColorAECMaxExposureTime(self, params=c_int32(0)):
        return self.sc_cam_lib.scSetColorAECMaxExposureTime(self.device_handle, params)

    def scGetColorAECMaxExposureTime(self):
        params = c_int32(0)
        return self.sc_cam_lib.scGetColorAECMaxExposureTime(self.device_handle, byref(params)), params.value

    def scGetMaxExposureTime(self, sensorType = ScSensorType.SC_COLOR_SENSOR):
        tmp = c_int32(1000)
        return self.sc_cam_lib.scGetMaxExposureTime(self.device_handle, sensorType.value, byref(tmp)), tmp.value

    def scSetHDRModeEnabled(self, enable =  c_bool(True)):
        return self.sc_cam_lib.scSetHDRModeEnabled(self.device_handle, enable)

    def scGetHDRModeEnabled(self):
        enable = c_bool(True)
        return self.sc_cam_lib.scGetHDRModeEnabled(self.device_handle, byref(enable)), enable.value

    def scGetFrameCountOfHDRMode(self):
        pCount = c_int32(0)
        return self.sc_cam_lib.scGetFrameCountOfHDRMode(self.device_handle, byref(pCount)), pCount.value

    def scSetExposureTimeOfHDR(self, frameIndex = c_uint8(0), exposureTime = c_int32(0)):
        return self.sc_cam_lib.scSetExposureTimeOfHDR(self.device_handle, frameIndex, exposureTime)

    def scGetExposureTimeOfHDR(self, frameIndex = c_uint8(0)):
        exposureTime = c_int32(0)
        return self.sc_cam_lib.scGetExposureTimeOfHDR(self.device_handle, frameIndex, byref(exposureTime)), exposureTime.value

    def scGetMaxExposureTimeOfHDR(self, frameIndex = c_uint8(0)):
        exposureTime = c_int32(0)
        return self.sc_cam_lib.scGetMaxExposureTimeOfHDR(self.device_handle, frameIndex, byref(exposureTime)), exposureTime.value

    def scSetWDRModeEnabled(self, enable =  c_bool(True)):
        return self.sc_cam_lib.scSetWDRModeEnabled(self.device_handle, enable)

    def scGetWDRModeEnabled(self):
        enable = c_bool(True)
        return self.sc_cam_lib.scGetWDRModeEnabled(self.device_handle, byref(enable)), enable.value

    def scGetFrameCountOfWDRMode(self):
        pCount = c_int32(0)
        return self.sc_cam_lib.scGetFrameCountOfWDRMode(self.device_handle, byref(pCount)), pCount.value

    def scSetExposureTimeOfWDR(self, frameIndex = c_uint8(0), exposureTime = c_int32(0)):
        return self.sc_cam_lib.scSetExposureTimeOfWDR(self.device_handle, frameIndex, exposureTime)

    def scGetExposureTimeOfWDR(self, frameIndex = c_uint8(0)):
        exposureTime = c_int32(0)
        return self.sc_cam_lib.scGetExposureTimeOfWDR(self.device_handle, frameIndex, byref(exposureTime)), exposureTime.value

    def scGetMaxExposureTimeOfWDR(self, frameIndex = c_uint8(0)):
        exposureTime = c_int32(0)
        return self.sc_cam_lib.scGetMaxExposureTimeOfWDR(self.device_handle, frameIndex, byref(exposureTime)), exposureTime.value

    def scSetTimeFilterParams(self, params = ScTimeFilterParams()): 
        return self.sc_cam_lib.scSetTimeFilterParams(self.device_handle, params)
    
    def scGetTimeFilterParams(self): 
        params = ScTimeFilterParams()
        return self.sc_cam_lib.scGetTimeFilterParams(self.device_handle, byref(params)),params

    def scSetConfidenceFilterParams(self, params = ScConfidenceFilterParams()): 
        return self.sc_cam_lib.scSetConfidenceFilterParams(self.device_handle, params)
    
    def scGetConfidenceFilterParams(self): 
        params = ScConfidenceFilterParams()
        return self.sc_cam_lib.scGetConfidenceFilterParams(self.device_handle, byref(params)),params
    
    def scSetFlyingPixelFilterParams(self, params = ScFlyingPixelFilterParams()): 
        return self.sc_cam_lib.scSetFlyingPixelFilterParams(self.device_handle, params)
    
    def scGetFlyingPixelFilterParams(self): 
        params = ScFlyingPixelFilterParams()
        return self.sc_cam_lib.scGetFlyingPixelFilterParams(self.device_handle, byref(params)),params

    def scSetFillHoleFilterEnabled(self, enable = c_bool(True)): 
        return self.sc_cam_lib.scSetFillHoleFilterEnabled(self.device_handle, enable)
    
    def scGetFillHoleFilterEnabled(self): 
        enable = c_bool(True)
        return self.sc_cam_lib.scGetFillHoleFilterEnabled(self.device_handle, byref(enable)),enable.value

    def scSetSpatialFilterEnabled(self, enable = c_bool(True)): 
        return self.sc_cam_lib.scSetSpatialFilterEnabled(self.device_handle, enable)
    
    def scGetSpatialFilterEnabled(self): 
        enable = c_bool(True)
        return self.sc_cam_lib.scGetSpatialFilterEnabled(self.device_handle, byref(enable)),enable.value

    def scSetTransformColorImgToDepthSensorEnabled(self, enabled = c_bool(True)): 
        return self.sc_cam_lib.scSetTransformColorImgToDepthSensorEnabled(self.device_handle,  enabled)
    
    def scGetTransformColorImgToDepthSensorEnabled(self): 
        enabled = c_bool(True)
        return self.sc_cam_lib.scGetTransformColorImgToDepthSensorEnabled(self.device_handle,  byref(enabled)),enabled.value

    def scSetTransformDepthImgToColorSensorEnabled(self, enabled = c_bool(True)): 
        return self.sc_cam_lib.scSetTransformDepthImgToColorSensorEnabled(self.device_handle,  enabled)
    
    def scGetTransformDepthImgToColorSensorEnabled(self): 
        enabled = c_bool(True)
        return self.sc_cam_lib.scGetTransformDepthImgToColorSensorEnabled(self.device_handle,  byref(enabled)),enabled.value

    def scTransformDepthPointToColorPoint(self, depthPoint = ScDepthVector3(), colorSize = ScVector2u16()):
        pPointInColor = ScVector2u16()
        return self.sc_cam_lib.scTransformDepthPointToColorPoint(self.device_handle, depthPoint, colorSize, byref(pPointInColor)), pPointInColor

    def scConvertDepthToPointCloud(self, pDepthVector = ScDepthVector3(), pointCount = c_int32(0), pSensorParam = ScSensorIntrinsicParameters()):
        tmp = ScVector3f * pointCount
        pWorldVector = tmp()
        return self.sc_cam_lib.scConvertDepthToPointCloud(self.device_handle, byref(pDepthVector), byref(pWorldVector), pointCount, byref(pSensorParam)), pWorldVector

    def scConvertDepthFrameToPointCloudVector(self, depthFrame = ScFrame()): 
        len = depthFrame.width*depthFrame.height
        tmp =ScVector3f*len
        pointlist = tmp()
        return self.sc_cam_lib.scConvertDepthFrameToPointCloudVector(self.device_handle, byref(depthFrame), byref(pointlist)), pointlist

    def scSetParamsByJson(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scSetParamsByJson(self.device_handle,  byref(path))

    def scExportParamInitFile(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scExportParamInitFile(self.device_handle, byref(path))

    def scImportParamInitFile(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scImportParamInitFile(self.device_handle, byref(path))

    def scRestoreParamInitFile(self):
        return self.sc_cam_lib.scRestoreParamInitFile(self.device_handle)

    def scRebootDevie(self):
        return self.sc_cam_lib.scRebootDevie(self.device_handle)

    def scSetHotPlugStatusCallback(self,callbackfunc= c_void_p): 
        callbackFunc_= ctypes.CFUNCTYPE(c_void_p,POINTER(ScDeviceInfo),c_int32)(callbackfunc)    
        gCallbackFuncList.append(callbackFunc_)
        return self.sc_cam_lib.scSetHotPlugStatusCallback(callbackFunc_)

    def scStartUpgradeFirmWare(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scStartUpgradeFirmWare(self.device_handle,  byref(path))

    def scGetUpgradeStatus(self):
        pStatus = c_int32(0)
        pUpgradeStatus = c_int32(0)
        return self.sc_cam_lib.scGetUpgradeStatus(self.device_handle, byref(pStatus), byref(pUpgradeStatus)), pStatus.value, pUpgradeStatus.value
    
    ### Morph related functions ###
    def scAIModuleSetEnabled(self, enable =  c_bool(True)):
        return self.sc_cam_lib.scAIModuleSetEnabled(self.device_handle, enable)

    def scAIModuleGetEnabled(self):
        enable = c_bool(True)
        return self.sc_cam_lib.scAIModuleGetEnabled(self.device_handle, byref(enable)), enable.value

    def scAIModuleSetParam(self, paramID = c_uint32(0), pBuffer = c_void_p(0), bufferSize = c_uint16(0)):
        return self.sc_cam_lib.scAIModuleSetParam(self.device_handle, paramID, pBuffer, bufferSize)

    def scAIModuleGetParam(self, paramID = c_uint32(0), pBuffer = c_void_p(0), bufferSize = c_uint16(0)):
        return self.sc_cam_lib.scAIModuleGetParam(self.device_handle, paramID, byref(pBuffer), byref(bufferSize))

    def scAIModuleSetWorkMode(self, mode = ScAIModuleMode(0)):
        return self.sc_cam_lib.scAIModuleSetWorkMode(self.device_handle, mode.value)

    def scAIModuleGetWorkMode(self):
        mode = c_uint8(0)
        return self.sc_cam_lib.scAIModuleGetWorkMode(self.device_handle, byref(mode)), mode.value

    def scAIModuleTriggerOnce(self):
        return self.sc_cam_lib.scAIModuleTriggerOnce(self.device_handle)

    def scAIModuleSetInputFrameTypeEnabled(self, frametype = ScFrameType.SC_DEPTH_FRAME, enable = c_bool(False)):
        return self.sc_cam_lib.scAIModuleSetInputFrameTypeEnabled(self.device_handle, frametype.value , enable)

    def scAIModuleGetInputFrameTypeEnabled(self ,frametype = ScFrameType.SC_DEPTH_FRAME):
        enable = c_bool(True)
        return self.sc_cam_lib.scAIModuleGetInputFrameTypeEnabled(self.device_handle, frametype.value, byref(enable)), enable.value

    def scAIModuleSetPreviewFrameTypeEnabled(self, frametype = ScFrameType.SC_DEPTH_FRAME, enable = c_bool(False)):
        return self.sc_cam_lib.scAIModuleSetPreviewFrameTypeEnabled(self.device_handle, frametype.value , enable)

    def scAIModuleGetPreviewFrameTypeEnabled(self ,frametype = ScFrameType.SC_DEPTH_FRAME):
        enable = c_bool(True)
        return self.sc_cam_lib.scAIModuleGetPreviewFrameTypeEnabled(self.device_handle, frametype.value, byref(enable)), enable.value

    def scAIModuleGetResult(self, waitTime = c_uint32(1000)):
        pAIResult = ScAIResult()
        return self.sc_cam_lib.scAIModuleGetResult(self.device_handle, waitTime, byref(pAIResult)), pAIResult

