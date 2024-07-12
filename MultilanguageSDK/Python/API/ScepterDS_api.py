from API.ScepterDS_define import * 
import platform
import os
import ctypes
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
        if system_ == 'linux':
            if machine_ == 'x86_64':
                os_info = os.uname()
                print('os_info:',type(os_info))
                system_info = os_info.version
                print('version:',system_info)
                if system_info.find('18.04') != -1 or system_info.find('20.04') != -1:
                    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Ubuntu18.04/Lib/libScepter_api.so"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Ubuntu16.04/Lib/libScepter_api.so"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
            elif machine_ == 'aarch64':
                libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/AArch64/Lib/libScepter_api.so"
                print(libpath)
                self.sc_cam_lib = cdll.LoadLibrary(libpath)
            else:
                print('do not supported OS', system_, machine_)
                exit()
        elif platform.system() == 'Windows':
            if machine_ == 'amd64':
                if architecture_ == '64bit':
                    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Windows/Bin/x64/Scepter_api.dll"
                    print(libpath)
                    self.sc_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = (os.path.abspath(os.path.dirname(os.getcwd()) + os.path.sep + "../../../../BaseSDK/"))+"/Windows/Bin/x86/Scepter_api.dll"
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
       
    def scSetWorkMode(self,  mode = ScWorkMode.SC_ACTIVE_MODE):
        return self.sc_cam_lib.scSetWorkMode(self.device_handle, mode.value) 
               
    def scGetWorkMode(self):
        mode = ScWorkMode(0)
        return self.sc_cam_lib.scGetWorkMode(self.device_handle, byref(mode)), mode

    def scSoftwareTriggerOnce(self):
        return self.sc_cam_lib.scSoftwareTriggerOnce(self.device_handle)
    
    def scRebootDevie(self):
        return self.sc_cam_lib.scRebootDevie(self.device_handle)
    
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
        return self.sc_cam_lib.scSetColorPixelFormat(self.device_handle, pixelFormat)

    def scSetColorResolution(self, w = c_int32(1600), h = c_int32(1200)):
        return self.sc_cam_lib.scSetColorResolution(self.device_handle, w, h)

    def scGetColorResolution(self):
        w = c_int32(1600)
        h = c_int32(1200)
        return self.sc_cam_lib.scGetColorResolution(self.device_handle, byref(w), byref(h)), w, h

    def scGetSupportedResolutionList(self, type = ScSensorType.SC_TOF_SENSOR, cam_count=1):
        tmp = ScResolutionList * cam_count
        pList = tmp()
        return self.sc_cam_lib.scGetSupportedResolutionList(self.device_handle, type, byref(pList)), pList

    def scSetFrameRate(self, value = c_uint8(30)):
        return self.sc_cam_lib.scSetFrameRate(self.device_handle, value) 
     
    def scGetFrameRate(self):
        value = c_uint8(1)
        return self.sc_cam_lib.scGetFrameRate(self.device_handle, byref(value)), value.value

    def scSetExposureControlMode(self, sensorType = ScSensorType.SC_TOF_SENSOR, mode = ScExposureControlMode.SC_EXPOSURE_CONTROL_MODE_MANUAL):
        return self.sc_cam_lib.scSetExposureControlMode(self.device_handle, sensorType.value, mode.value) 

    def scGetExposureControlMode(self, sensorType = ScSensorType.SC_TOF_SENSOR):
        mode = ScExposureControlMode(1)
        return self.sc_cam_lib.scGetExposureControlMode(self.device_handle, sensorType.value, byref(mode)), mode

    def scSetExposureTime(self, sensorType = ScSensorType.SC_TOF_SENSOR, params = c_int32(0)):
        return self.sc_cam_lib.scSetExposureTime(self.device_handle, sensorType.value, params)
     
    def scGetExposureTime(self, sensorType = ScSensorType.SC_TOF_SENSOR):
        params = c_int32(0)
        return self.sc_cam_lib.scGetExposureTime(self.device_handle, sensorType.value, byref(params)), params

    def scSetColorAECMaxExposureTime(self, params=c_int32(0)):
        return self.sc_cam_lib.scSetColorAECMaxExposureTime(self.device_handle, params)

    def scGetColorAECMaxExposureTime(self):
        params = c_int32(0)
        return self.sc_cam_lib.scGetColorAECMaxExposureTime(self.device_handle, byref(params)), params

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
        return self.sc_cam_lib.scGetTransformColorImgToDepthSensorEnabled(self.device_handle,  byref(enabled)),enabled

    def scSetTransformDepthImgToColorSensorEnabled(self, enabled = c_bool(True)): 
        return self.sc_cam_lib.scSetTransformDepthImgToColorSensorEnabled(self.device_handle,  enabled)
    
    def scGetTransformDepthImgToColorSensorEnabled(self): 
        enabled = c_bool(True)
        return self.sc_cam_lib.scGetTransformDepthImgToColorSensorEnabled(self.device_handle,  byref(enabled)),enabled

    def scTransformDepthPointToColorPoint(self, depthPoint = ScDepthVector3(), colorSize = ScVector2u16()):
        pPointInColor = ScVector2u16()
        return self.sc_cam_lib.scTransformDepthPointToColorPoint(self.device_handle, depthPoint, colorSize, byref(PointInColor)), pPointInColor

    def scConvertDepthToPointCloud(self, pDepthVector = ScDepthVector3(), pointCount = c_int32(0), pSensorParam = ScSensorIntrinsicParameters()):
        tmp = ScVector3f * pointCount
        pWorldVector = tmp()
        return self.sc_cam_lib.scConvertDepthToPointCloud(self.device_handle, byref(pDepthVector), byref(pWorldVector), byref(pSensorParam)),pWorldVector

    def scConvertDepthFrameToPointCloudVector(self, depthFrame = ScFrame()): 
        len = depthFrame.width*depthFrame.height
        tmp =ScVector3f*len
        pointlist = tmp()
        return self.sc_cam_lib.scConvertDepthFrameToPointCloudVector(self.device_handle, byref(depthFrame) ,pointlist),pointlist

    def scSetHotPlugStatusCallback(self,callbackfunc= c_void_p): 
        callbackFunc_= ctypes.CFUNCTYPE(c_void_p,POINTER(ScDeviceInfo),c_int32)(callbackfunc)    
        gCallbackFuncList.append(callbackFunc_)
        return self.sc_cam_lib.scSetHotPlugStatusCallback(callbackFunc_)

    def scGetMaxExposureTime(self, sensorType = ScSensorType.SC_COLOR_SENSOR):
        tmp = c_int32(1000)
        return self.sc_cam_lib.scGetMaxExposureTime(self.device_handle, sensorType.value, byref(tmp)), tmp

    def scSetParamsByJson(self, imgpath):
        pimgpath = (c_char * 1000)(*bytes(imgpath, 'utf-8'))
        return self.sc_cam_lib.scSetParamsByJson(self.device_handle,  byref(pimgpath))

    def scSetColorGain(self, params = c_float(1.0)):
        return self.sc_cam_lib.scSetColorGain(self.device_handle,  params)

    def scGetColorGain(self):
        tmp = c_float*1
        params = tmp()
        return self.sc_cam_lib.scGetColorGain(self.device_handle,  params), params

    def scSetAutoExposureTime(self, params = c_int32(0)):
        return self.sc_cam_lib.scSetColorAECMaxExposureTime(self.device_handle, params)

    def scGetAutoExposureTime(self):
        params = c_int32(0)
        return self.sc_cam_lib.scGetColorAECMaxExposureTime(self.device_handle, byref(params)), params

    def scSetInputSignalParamsForHWTrigger(self, params = ScInputSignalParamsForHWTrigger()):
        return self.sc_cam_lib.scSetInputSignalParamsForHWTrigger(self.device_handle, params)

    def scGetInputSignalParamsForHWTrigger(self):
        params = ScInputSignalParamsForHWTrigger()
        return self.sc_cam_lib.scGetInputSignalParamsForHWTrigger(self.device_handle, byref(params)), params

    def scGetDepthRangeValue(self):
        minValue = c_int16(0)
        maxValue = c_int16(0)
        return self.sc_cam_lib.scGetDepthRangeValue(self.device_handle, byref(minValue), byref(maxValue)), minValue, maxValue

    def scSetTimeSyncConfig(self, params = ScTimeSyncConfig()):
        return self.sc_cam_lib.scSetTimeSyncConfig(self.device_handle, params)

    def scGetTimeSyncConfig(self):
        pParams = ScTimeSyncConfig()
        return self.sc_cam_lib.scGetTimeSyncConfig(self.device_handle, byref(pParams)),pParams

    def scSetHDRModeEnabled(self, bEnabled =  c_bool(True)):
        return self.sc_cam_lib.scSetHDRModeEnabled(self.device_handle, bEnabled)

    def scGetHDRModeEnabled(self):
        bEnabled = c_bool(True)
        return self.sc_cam_lib.scGetHDRModeEnabled(self.device_handle, byref(bEnabled)), bEnabled.value

    def scGetDistanceLevelCountOfHDRMode(self):
        pCount = c_int32(0)
        return self.sc_cam_lib.scGetDistanceLevelCountOfHDRMode(self.device_handle, byref(pCount)), pCount

    def scSetExposureTimeOfHDR(self, level = c_uint8(0), exposureTime = c_int32(0)):
        return self.sc_cam_lib.scSetExposureTimeOfHDR(self.device_handle, level, exposureTime)

    def scGetExposureTimeOfHDR(self):
        level = c_uint8(0)
        exposureTime = c_int32(0)
        return self.sc_cam_lib.scGetExposureTimeOfHDR(self.device_handle, byref(level), byref(exposureTime)), level, exposureTime

    def scGetMaxExposureTimeOfHDR(self):
        level = c_uint8(0)
        pMaxExposureTime = c_int32(0)
        return self.sc_cam_lib.scGetMaxExposureTimeOfHDR(self.device_handle, byref(level), byref(pMaxExposureTime)), level, pMaxExposureTime

    def scExportParamInitFile(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scExportParamInitFile(self.device_handle, byref(path))

    def scImportParamInitFile(self, pfilePath):
        path = (c_char * 1000)(*bytes(pfilePath, 'utf-8'))
        return self.sc_cam_lib.scImportParamInitFile(self.device_handle, byref(path))

    def scRestoreParamInitFile(self, pfilePath):
        return self.sc_cam_lib.scRestoreParamInitFile(self.device_handle)