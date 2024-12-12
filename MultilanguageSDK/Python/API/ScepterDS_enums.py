import os, platform, numpy
from ctypes import *
from enum import Enum

class ScFrameType(Enum):
    SC_DEPTH_FRAME       = 0     
    SC_IR_FRAME          = 1
    SC_COLOR_FRAME      = 3
    SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME = 4
    SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME = 5
        
class ScSensorType(Enum):
    SC_TOF_SENSOR = 0x01
    SC_COLOR_SENSOR = 0x02
    
class ScPixelFormat(Enum):
    SC_PIXEL_FORMAT_DEPTH_MM16      = 0
    SC_PIXEL_FORMAT_GRAY_8          = 2
    SC_PIXEL_FORMAT_RGB_888_JPEG    = 3
    SC_PIXEL_FORMAT_BGR_888_JPEG    = 4
    SC_PIXEL_FORMAT_RGB_888         = 5
    SC_PIXEL_FORMAT_BGR_888         = 6
    SC_PIXEL_FORMAT_RGB_565         = 7
    SC_PIXEL_FORMAT_BGR_565         = 8

class ScReturnStatus(Enum):
    SC_OK                           =  0
    SC_NO_DEVICE_CONNECTED          = -1
    SC_INVALID_DEVICE_INDEX         = -2
    SC_DEVICE_POINTER_IS_NULL       = -3
    SC_INVALID_FRAME_TYPE           = -4
    SC_FRAME_POINTER_IS_NULL        = -5
    SC_NO_PROPERTY_VALUE_GET        = -6
    SC_NO_PROPERTY_VALUE_SET        = -7
    SC_PROPERTY_POINTER_IS_NULL     = -8
    SC_PROPERTY_SIZE_NOT_ENOUGH     = -9
    SC_INVALID_DEPTH_RANGE          = -10
    SC_GET_FRAME_READY_TIME_OUT     = -11
    SC_INPUT_POINTER_IS_NULL        = -12
    SC_CAMERA_NOT_OPENED            = -13
    SC_INVALID_CAMERA_TYPE          = -14
    SC_INVALID_PARAMS               = -15
    SC_CURRENT_VERSION_NOT_SUPPORT  = -16
    SC_UPGRADE_IMG_ERROR            = -17
    SC_UPGRADE_IMG_PATH_TOO_LONG    = -18
    SC_UPGRADE_CALLBACK_NOT_SET		= -19
    SC_PRODUCT_NOT_SUPPORT          = -20
    SC_NO_CONFIG_FOLDER             = -21
    SC_WEB_SERVER_START_ERROR       = -22
    SC_GET_OVER_STAY_FRAME          = -23
    SC_CREATE_LOG_DIR_ERROR         = -24
    SC_CREATE_LOG_FILE_ERROR        = -25
    SC_NO_ADAPTER_CONNECTED			= -100
    SC_REINITIALIZED				= -101
    SC_NO_INITIALIZED				= -102
    SC_CAMERA_OPENED				= -103
    SC_CMD_ERROR					= -104
    SC_CMD_SYNC_TIME_OUT	        = -105
    SC_IP_NOT_MATCH					= -106
    SC_NOT_STOP_STREAM              = -107
    SC_NOT_START_STREAM             = -108
    SC_NOT_FIND_DRIVERS_FOLDER      = -109
    SC_CAMERA_OPENING               = -110
    SC_CAMERA_OPENED_BY_ANOTHER_APP = -111
    ScRetOthers                     = -255
    
class ScConnectStatus(Enum):
    SC_LIMBO            = 0
    SC_CONNECTABLE      = 1
    SC_OPENED           = 2


class ScWorkMode(Enum):
    SC_ACTIVE_MODE           = 0x00
    SC_HARDWARE_TRIGGER_MODE = 0x01
    SC_SOFTWARE_TRIGGER_MODE = 0x02

class ScExposureControlMode(Enum):
    SC_EXPOSURE_CONTROL_MODE_AUTO   = 0
    SC_EXPOSURE_CONTROL_MODE_MANUAL = 1

class ScAIModuleMode(Enum):
    AI_CONTINUOUS_RUN_MODE= 0x00
    AI_SINGLE_RUN_MODE = 0x01
    

