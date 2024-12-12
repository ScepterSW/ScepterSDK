import os, platform, numpy
from ctypes import *
from enum import Enum

class ScVector3f(Structure):
    _pack_ = 1
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]  

class ScVector2u16(Structure):
    _pack_ = 1
    _fields_ = [("x", c_uint16),
                ("y", c_uint16)]  

class ScDepthVector3(Structure):
    _pack_ = 1
    _fields_ = [("depthX", c_int),
                ("depthY", c_int),
                ("depthZ", c_uint16)]

class ScResolution(Structure):
    _pack_ = 1
    _fields_ = [("width", c_int),
                ("height", c_int)]

class ScResolutionList(Structure):
    _pack_ = 1
    _fields_ = [("count", c_int),
                ("resolution", ScResolution * 6)]

class ScSensorIntrinsicParameters(Structure):
    _pack_ = 1
    _fields_ = [("fx", c_double),
                ("fy", c_double),
                ("cx", c_double),
                ("cy", c_double),
                ("k1", c_double),
                ("k2", c_double),
                ("p1", c_double),
                ("p2", c_double),
                ("k3", c_double),
                ("k4", c_double),
                ("k5", c_double),
                ("k6", c_double)] 

class ScSensorExtrinsicParameters(Structure):
    _pack_ = 1
    _fields_ = [("rotation", c_double * 9),
                ("translation", c_double * 3)]

class ScFrame(Structure):
    _pack_ = 1
    _fields_ = [("frameIndex", c_uint32),
                ("frameType", c_int32),
                ("pixelFormat", c_int32),
                ("pFrameData", POINTER(c_uint8)),
                ("dataLen", c_uint32),
                ("width", c_uint16),
                ("height", c_uint16),
                ("hardwaretimestamp", c_uint64)]

class ScFrameReady(Structure):
    _pack_ = 1
    _fields_ = [("depth", c_uint, 1),
                ("ir", c_uint, 1),
                ("color", c_uint, 1),
                ("transformedColor", c_uint, 1),
                ("transformedDepth", c_uint, 1),
                ("reserved", c_uint, 27)]

class ScDeviceInfo(Structure):
    _pack_ = 1
    _fields_ = [("productName", c_char * 64),
                ("serialNumber", c_char * 64),
                ("ip", c_char * 17),
                ("status", c_int32)]

class ScConfidenceFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("threshold", c_int32),
                ("enable", c_bool)]

class ScFlyingPixelFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("threshold", c_int32),
                ("enable", c_bool)]

class ScTimeFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("threshold", c_int32),
                ("enable", c_bool)]

class ScIRGMMCorrectionParams(Structure):
    _pack_ = 1
    _fields_ = [("threshold", c_int32),
                ("enable", c_bool)]

class ScInputSignalParamsForHWTrigger(Structure):
    _pack_ = 1
    _fields_ = [("width", c_uint16),
                ("interval", c_uint16),
                ("polarity", c_uint8)]

class ScOutputSignalParams(Structure):
    _pack_ = 1
    _fields_ = [("width", c_uint16),
                ("delay", c_uint16),
                ("polarity", c_uint8)]

class ScTimeSyncConfig(Structure):
    _pack_ = 1
    _fields_ = [("flag", c_uint8),
                ("ip", c_uint8 * 16)]

class ScAIResult(Structure):
    _pack_ = 1
    _fields_ = [("resultIndex", c_uint32),
                ("pResultData",POINTER(c_uint8)),
                ("dataLen", c_uint32),
                ("resultTimestamp", c_uint64)]

