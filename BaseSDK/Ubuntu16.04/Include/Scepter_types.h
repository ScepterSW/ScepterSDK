#ifndef SCEPTER_TYPES_H
#define SCEPTER_TYPES_H

#include <stdint.h>
#include "Scepter_enums.h"

#ifndef __cplusplus
    #include <stdbool.h>
#endif

typedef uint16_t ScDepthPixel;   //!< Depth image pixel type in 16-bit.

#pragma pack(push, 1)

/**
 * @brief Stores the x, y, and z components of a 3D vector.
 */
typedef struct
{
    float x;   //!< The x components of the vector.
    float y;   //!< The y components of the vector.
    float z;   //!< The z components of the vector.
} ScVector3f;

/**
 * @brief Stores the x and y components of a 2D vector.
 */
typedef struct
{
    uint16_t x;   //!< The x components of the vector.
    uint16_t y;   //!< The y components of the vector.
} ScVector2u16;

/**
 * @brief Contains depth information for a given pixel.
 */
typedef struct
{
    int32_t      depthX;   //!< The x coordinate of the pixel.
    int32_t      depthY;   //!< The y coordinate of the pixel.
    ScDepthPixel depthZ;   //!< The depth of the pixel, in millimeters.
} ScDepthVector3;

/**
 * @brief image resolution.
 */
typedef struct
{
    int32_t width;
    int32_t height;
} ScResolution;

/**
 * @brief Supported resolutions.
 */
typedef struct
{
    int32_t      count;
    ScResolution resolution[6];
} ScResolutionList;

/**
 * @brief Camera intrinsic parameters and distortion coefficients.
 */
typedef struct
{
    double fx;   //!< Focal length x (pixel).
    double fy;   //!< Focal length y (pixel).
    double cx;   //!< Principal point x (pixel).
    double cy;   //!< Principal point y (pixel).
    double k1;   //!< Radial distortion coefficient, 1st-order.
    double k2;   //!< Radial distortion coefficient, 2nd-order.
    double p1;   //!< Tangential distortion coefficient.
    double p2;   //!< Tangential distortion coefficient.
    double k3;   //!< Radial distortion coefficient, 3rd-order.
    double k4;   //!< Radial distortion coefficient, 4st-order.
    double k5;   //!< Radial distortion coefficient, 5nd-order.
    double k6;   //!< Radial distortion coefficient, 6rd-order.
} ScSensorIntrinsicParameters;

/**
 * @brief Extrinsic parameters defines the physical relationship form tof sensor to color sensor.
 */
typedef struct
{
    double rotation[9];      //!< Orientation stored as an array of 9 double representing a 3x3 rotation matrix.
    double translation[3];   //!< Location stored as an array of 3 double representing a 3-D translation vector.
} ScSensorExtrinsicParameters;

/**
 * @brief Depth/IR/Color image frame data.
 */
typedef struct
{
    uint32_t      frameIndex;        //!< The index of the frame.
    ScFrameType   frameType;         //!< The type of frame. See ::ScFrameType for more information.
    ScPixelFormat pixelFormat;       //!< The pixel format used by a frame. See ::ScPixelFormat for more information.
    uint8_t*      pFrameData;        //!< A buffer containing the frame’s image data.
    uint32_t      dataLen;           //!< The length of pFrame, in bytes.
    uint16_t      width;             //!< The width of the frame, in pixels.
    uint16_t      height;            //!< The height of the frame, in pixels.
    uint64_t      deviceTimestamp;   //!< The timestamp(in milliseconds) when the frame be generated on the device. Frame processing and transfer time are not included.
} ScFrame;

typedef struct
{
    uint32_t depth            : 1;    //!< Whether the depth image is ready.
    uint32_t ir               : 1;    //!< Whether the ir image is ready.
    uint32_t color            : 1;    //!< Whether the color image is ready.
    uint32_t transformedColor : 1;    //!< Whether the transformedColor image is ready.
    uint32_t transformedDepth : 1;    //!< Whether the transformedDepth image is ready.
    uint32_t reserved         : 27;   //!< Placeholder.
} ScFrameReady;

typedef void* ScDeviceHandle;

typedef struct
{
    char            productName[64];    //!< Product type name.
    char            serialNumber[64];   //!< Device serial number.
    char            ip[17];             //!< Device IP.
    ScConnectStatus status;             //!< Device status.
} ScDeviceInfo;

typedef struct
{
    int32_t threshold;   //!< Range in [1, 6],The larger the value is, the more obvious the filtering effect is and The smaller the point cloud wobble.
    bool    enable;      //!< Whether to enable time filter.
} ScTimeFilterParams;

typedef struct
{
    int32_t threshold;   //!< Range in [1, 100]. The larger the value is, the more obvious the filtering effect is and the more points are filtered out.
    bool    enable;      //!< Whether to enable confidence filter.
} ScConfidenceFilterParams;

typedef struct
{
    int32_t threshold;   //!< Range in [1, 16]. The larger the value is, the more obvious the filtering effect is and the more points are filtered out.
    bool    enable;      //!< Whether to enable flying pixel filter.
} ScFlyingPixelFilterParams;

typedef struct
{
    int32_t threshold;   //!< Range in [1, 100]. The larger the value is, the more obvious the correction effect.
    bool    enable;      //!< Whether to enable IRGMM correction.
} ScIRGMMCorrectionParams;

typedef struct
{
    uint16_t width;                  //!< Range in [1,65535]. The width of input signal.
    uint16_t interval;               //!< Range in [34000,65535]. The interval of input signal.
    uint8_t  polarity;               //!< Range in [0,1]. 0 for active low; 1 for active high.
} ScInputSignalParamsForHWTrigger;   //!< Input signal parameters for Hardware Trigger.

typedef struct
{
    uint16_t width;       //!< Range in [1,65535]. The width of output signal.
    uint16_t delay;       //!< Range in [0,65535]. The delay time of output signal.
    uint8_t  polarity;    //!< Range in [0,1]. 0 for active low; 1 for active high.
} ScOutputSignalParams;   //!< Output signal parameters.

typedef struct
{
    uint8_t flag;     //!< 0: disable, 1: NTP, 2: PTP, only NTP needs the ip.
    uint8_t ip[16];   //!< just for NTP.
} ScTimeSyncConfig;

#pragma pack(pop)

/**
 * @brief         Hot plug status callback function.
 * @param[out]    pInfo     Return the info of the Device, See ::ScDeviceInfo.
 * @param[out]    state     Hot plug status. 0:device added , 1:device removed.
 * @param[out]    pUserData Pointer to user data, which can be null.
 */
typedef void (*PtrHotPlugStatusCallback)(const ScDeviceInfo* pInfo, int state, void* pUserData);

/**
 * @brief         Upgrade status callback function.
 * @param[out]    status     Returns the upgrade step status.
 * @param[out]    params     Params of upgrade step status , -1:upgrade fail , 0:upgrade Normal, 1-100:upgrade progress.
 * @param[out]    pUserData  Pointer to user data, which can be null.
 */
typedef void (*PtrUpgradeStatusCallback)(int status, int params, void* pUserData);

#endif /* SCEPTER_TYPES_H */
