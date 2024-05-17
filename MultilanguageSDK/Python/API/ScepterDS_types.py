import os, platform, numpy
from ctypes import *
from enum import Enum
 

class ScRGB888Pixel(Structure):
    _pack_ = 1
    _fields_ = [("r", c_uint8),
                ("g", c_uint8),
                ("b", c_uint8)]

class ScBGR888Pixel(Structure):
    _pack_ = 1
    _fields_ = [("b", c_uint8),
                ("g", c_uint8),
                ("r", c_uint8)]

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

class ScTimeStamp(Structure):
    _pack_ = 1       
    _fields_ = [("tm_sec", c_uint16),
                ("tm_min", c_uint16),
                ("tm_hour", c_uint16),
                ("tm_msec", c_uint16)]     

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


