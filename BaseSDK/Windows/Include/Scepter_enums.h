#ifndef SCEPTER_ENUMS_H
#define SCEPTER_ENUMS_H

/**
 * @brief    Specifies the type of image frame.
 */
typedef enum
{
    SC_DEPTH_FRAME                               = 0,   //!< Depth frame with 16 bits per pixel in millimeters.
    SC_IR_FRAME                                  = 1,   //!< IR frame with 8 bits per pixel.
    SC_COLOR_FRAME                               = 3,   //!< Color frame with 24 bits per pixel in RGB/BGR format.
    SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME = 4,   //!< Color frame with 24 bits per pixel in RGB/BGR format, that is transformed to depth
                                                        //!< sensor space where the resolution is the same as the depth frame's resolution.
                                                        //!< This frame type can be enabled using ::scSetTransformColorImgToDepthSensorEnabled().
    SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME = 5,   //!< Depth frame with 16 bits per pixel, in millimeters, that is transformed to color sensor
                                                        //!< space where the resolution is same as the color frame's resolution.
                                                        //!< This frame type can be enabled using ::scSetTransformDepthImgToColorSensorEnabled().
} ScFrameType;

/**
 * @brief    Specifies the image pixel format.
 */
typedef enum
{
    SC_PIXEL_FORMAT_DEPTH_MM16   = 0,   //!< Depth image pixel format, 16 bits per pixel in mm.
    SC_PIXEL_FORMAT_GRAY_8       = 2,   //!< Gray image pixel format, 8 bits per pixel.

    // Color
    SC_PIXEL_FORMAT_RGB_888_JPEG = 3,   //!< By jpeg decompress, color image pixel format, 24 bits per pixel RGB format.
    SC_PIXEL_FORMAT_BGR_888_JPEG = 4,   //!< By jpeg decompress, color image pixel format, 24 bits per pixel BGR format.
    SC_PIXEL_FORMAT_RGB_888      = 5,   //!< Without compress, color image pixel format, 24 bits per pixel RGB format.
    SC_PIXEL_FORMAT_BGR_888      = 6,   //!< Without compress, color image pixel format, 24 bits per pixel BGR format.
    SC_PIXEL_FORMAT_RGB_565      = 7,   //!< Without compress, color image pixel format, 16 bits per pixel RGB format.
    SC_PIXEL_FORMAT_BGR_565      = 8,   //!< Without compress, color image pixel format, 16 bits per pixel BGR format.
} ScPixelFormat;

/**
 * @brief    Specifies the type of sensor.
 */
typedef enum
{
    SC_TOF_SENSOR   = 0x01,   //!< ToF camera.
    SC_COLOR_SENSOR = 0x02,   //!< Color camera.
} ScSensorType;

/**
 * @brief    Return status codes for all APIs.
 *           <code>SC_OK = 0</code> means the API successfully completed its operation.
 *           All other codes indicate a device, parameter, or API usage error.
 */
typedef enum
{
    SC_OK                           = 0,      //!< The function completed successfully.
    SC_DEVICE_IS_LIMBO              = -1,     //!< The device is limbo
    SC_INVALID_DEVICE_INDEX         = -2,     //!< The input device index is invalid.
    SC_DEVICE_POINTER_IS_NULL       = -3,     //!< The device structure pointer is null.
    SC_INVALID_FRAME_TYPE           = -4,     //!< The input frame type is invalid.
    SC_FRAME_POINTER_IS_NULL        = -5,     //!< The output frame buffer is null.
    SC_NO_PROPERTY_VALUE_GET        = -6,     //!< Cannot get the value for the specified property.
    SC_NO_PROPERTY_VALUE_SET        = -7,     //!< Cannot set the value for the specified property.
    SC_PROPERTY_POINTER_IS_NULL     = -8,     //!< The input property value buffer pointer is null.
    SC_PROPERTY_SIZE_NOT_ENOUGH     = -9,     //!< The input property value buffer size is too small to store the specified property value.
    SC_INVALID_DEPTH_RANGE          = -10,    //!< The input depth range mode is invalid.
    SC_GET_FRAME_READY_TIME_OUT     = -11,    //!< Capture the next image frame time out.
    SC_INPUT_POINTER_IS_NULL        = -12,    //!< An input pointer parameter is null.
    SC_CAMERA_NOT_OPENED            = -13,    //!< The camera has not been opened.
    SC_INVALID_CAMERA_TYPE          = -14,    //!< The specified type of camera is invalid.
    SC_INVALID_PARAMS               = -15,    //!< One or more of the parameter values provided are invalid.
    SC_CURRENT_VERSION_NOT_SUPPORT  = -16,    //!< This feature is not supported in the current version.
    SC_UPGRADE_IMG_ERROR            = -17,    //!< There is an error in the upgrade file.
    SC_UPGRADE_IMG_PATH_TOO_LONG    = -18,    //!< Upgrade file path length greater than 260.
    SC_UPGRADE_CALLBACK_NOT_SET     = -19,    //!< scSetUpgradeStatusCallback is not called.
    SC_PRODUCT_NOT_SUPPORT          = -20,    //!< The current product does not support this operation.
    SC_NO_CONFIG_FOLDER             = -21,    //!< No product profile found.
    SC_WEB_SERVER_START_ERROR       = -22,    //!< WebServer Start/Restart error(IP or PORT 8080).
    SC_GET_OVER_STAY_FRAME          = -23,    //!< The time from frame ready to get frame is out of 1s.
    SC_CREATE_LOG_DIR_ERROR         = -24,    //!< Create log directory error.
    SC_CREATE_LOG_FILE_ERROR        = -25,    //!< Create log file error.
    SC_NO_ADAPTER_CONNECTED         = -100,   //!< There is no adapter connected.
    SC_REINITIALIZED                = -101,   //!< The SDK has been Initialized.
    SC_NO_INITIALIZED               = -102,   //!< The SDK has not been Initialized.
    SC_CAMERA_OPENED                = -103,   //!< The camera has been opened.
    SC_CMD_ERROR                    = -104,   //!< Set/Get cmd control error.
    SC_CMD_SYNC_TIME_OUT            = -105,   //!< Set cmd ok.but time out for the sync return.
    SC_IP_NOT_MATCH                 = -106,   //!< IP is not in the same network segment.
    SC_NOT_STOP_STREAM              = -107,   //!< Please invoke scStopStream first to close the data stream.
    SC_NOT_START_STREAM             = -108,   //!< Please invoke scStartStream first to get the data stream.
    SC_NOT_FIND_DRIVERS_FOLDER      = -109,   //!< Please check whether the Drivers directory exists.
    SC_CAMERA_OPENING               = -110,   //!< The camera is openin,by another Sc_OpenDeviceByXXX API.
    SC_CAMERA_OPENED_BY_ANOTHER_APP = -111,   //!< The camera has been opened by another APP.
    SC_GET_AI_RESULT_TIME_OUT       = -112,   //!< Capture the next AI result time out.
    SC_MORPH_AI_LIB_ERROR           = -113,   //!< The morph Al library is not exist or initialized failed.
    SC_CPU_AFFINITY_CHECK_FAILED    = -114,   //!< The cpu affinity config file check failed

    SC_OTHERS                       = -255,   //!< An unknown error occurred.
} ScStatus;

typedef enum
{
    SC_LIMBO       = 0,   //!< Unknown device status and cannot try to open.
    SC_CONNECTABLE = 1,   //!< Device connectable state and support open.
    SC_OPENED      = 2,   //!< The device is connected and cannot be opened again.
} ScConnectStatus;

typedef enum
{
    SC_ACTIVE_MODE           = 0x00,   //!< Enter the active mode.
    SC_HARDWARE_TRIGGER_MODE = 0x01,   //!< Enter the hardware salve mode, at this time need to connect
                                       //!< the hardware trigger wire, provide hardware signal, to trigger the image.
    SC_SOFTWARE_TRIGGER_MODE = 0x02,   //!< Enter the software salve mode, at this time need to invoke scSoftwareTriggerOnce, to trigger the image.
} ScWorkMode;

typedef enum
{
    SC_EXPOSURE_CONTROL_MODE_AUTO   = 0,   //!< Enter the auto exposure mode.
    SC_EXPOSURE_CONTROL_MODE_MANUAL = 1,   //!< Enter the manual exposure mode.
} ScExposureControlMode;

#endif /* SCEPTER_ENUMS_H */
