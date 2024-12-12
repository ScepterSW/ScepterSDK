#ifndef SCEPTER_API_H
#define SCEPTER_API_H

/**
 * @file Scepter_api.h
 * @brief Scepter API header file.
 * Copyright (c) 2024 Goermicro Inc.
 */

/*! \mainpage Scepter API Documentation
 *
 * \section intro_sec Introduction
 *
 * Welcome to the Scepter API documentation. This documentation enables you to quickly get started in your
 * development efforts to programmatically interact with the Scepter CW ToF Camera.
 */

#include "Scepter_define.h"

/**
 * @brief        Initializes the API on the device. This function must be invoked before any other Scepter APIs.
 * @return       ::SC_OK    If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scInitialize();

/**
 * @brief        Shuts down the API on the device and clears all resources allocated by the API. After
 *               invoking this function, no other Scepter APIs can be invoked.
 * @return       ::SC_OK    If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scShutdown();

/**
 * @brief        Get the version of SDK.
 * @return       Returns sdk version.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSDKVersion(char* pSDKVersion, int32_t length);

/**
 * @brief        Returns the number of camera devices currently connected.
 * @param[out]   pDeviceCount    Pointer to a 32-bit integer variable in which to return the device count.
 * @param[in]    scanTime        Scans time, the unit is millisecond.
                                 This function scans devices for scanTime(ms) and then returns the count of devices.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceCount(uint32_t* pDeviceCount, uint32_t scanTime);

/**
 * @brief        Returns the info lists of the deviceCount camera devices.
 * @param[in]    deviceCount         The number of camera devices.
 * @param[out]   pDevicesInfoList    Pointer to a buffer in which to store the devices list infos.
 * @return       ::SC_OK             If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceInfoList(uint32_t deviceCount, ScDeviceInfo* pDevicesInfoList);

/**
 * @brief        Opens the device specified by <code>serialNumber</code>. The device must be subsequently closed using scCloseDevice().
 * @param[in]    pSN         The uri of the device. See ::ScDeviceInfo for more information.
 * @param[out]   pDevice      The handle of the device on which to open.
 * @return:      ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scOpenDeviceBySN(const char* pSN, ScDeviceHandle* pDevice);

/**
 * @brief        Opens the device specified by <code>ip</code>. The device must be subsequently closed using scCloseDevice().
 * @param[in]    pIP          The ip of the device. See ::ScDeviceInfo for more information.
 * @param[out]   pDevice      The handle of the device on which to open.
 * @return:      ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scOpenDeviceByIP(const char* pIP, ScDeviceHandle* pDevice);

/**
 * @brief        Closes the device specified by <code>device</code> that was opened using scOpenDevice.
 * @param[in]    pDevice       The handle of the device to close.
 * @return:      ::SC_OK       If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scCloseDevice(ScDeviceHandle* pDevice);

/**
 * @brief        Starts capturing the image stream indicated by <code>device</code>. Invoke scStopStream() to stop capturing the image stream.
 * @param[in]    device          The handle of the device on which to start capturing the image stream.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scStartStream(ScDeviceHandle device);

/**
 * @brief        Stops capturing the image stream on the device specified by <code>device</code>. that was started using scStartStream.
 * @param[in]    device       The handle of the device on which to stop capturing the image stream.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scStopStream(ScDeviceHandle device);

/**
 * @brief        Captures the next image frame from the device specified by <code>device</code>. This API must be invoked before capturing frame data using scGetFrame().
 * @param[in]    device         The handle of the device on which to read the next frame.
 * @param[in]    waitTime       The unit is millisecond, the value is in the range (0,65535).
 *                              You can change the value according to the frame rate. For example,the frame rate is 30, so the theoretical waittime interval is 33ms,
 *                              but if set the time value is 20ms, it means the maximum wait time is 20 ms when capturing next frame, so when call the scGetFrameReady,
 *                              it may return SC_GET_FRAME_READY_TIME_OUT(-11).
 *                              So the recommended value is 2 * 1000/ FPS.
 * @param[out]   pFrameReady    Pointer to a buffer in which to store the signal on which image is ready to be get.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFrameReady(ScDeviceHandle device, uint16_t waitTime, ScFrameReady* pFrameReady);

/**
 * @brief        Returns the image data for the current frame from the device specified by <code>device</code>.
 *               Before invoking this API, invoke scGetFrameReady() to capture one image frame from the device.
 * @param[in]    device       The handle of the device to capture an image frame from.
 * @param[in]    frameType    The image frame type.
 * @param[out]   pScFrame     Pointer to a buffer in which to store the returned image data.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFrame(ScDeviceHandle device, ScFrameType frameType, ScFrame* pScFrame);

/**
 * @brief        Get the depth range in the current working mode of the device.
 * @param[in]    device       The handle of the device.
 * @param[out]   minValue     The min value of the depth
 * @param[out]   maxValue     The ax value of the depth
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDepthRangeValue(ScDeviceHandle device, int16_t* minValue, int16_t* maxValue);

/**
 * @brief        Returns the internal intrinsic and distortion coefficient parameters from the device specified by <code>device</code>.
 * @param[in]    device                        The handle of the device from which to get the internal parameters.
 * @param[in]    sensorType                    The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[out]   pSensorIntrinsicParameters    Pointer to a ScSensorIntrinsicParameters variable in which to store the parameter values.
 * @return       ::SC_OK                       If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSensorIntrinsicParameters(ScDeviceHandle device, ScSensorType sensorType, ScSensorIntrinsicParameters* pSensorIntrinsicParameters);

/**
 * @brief        Returns the camera rotation and translation coefficient parameters from the device specified by <code>device</code>.
 * @param[in]    device                        The handle of the device from which to get the extrinsic parameters.
 * @param[out]   pSensorExtrinsicParameters    Pointer to a ::ScSensorExtrinsicParameters variable in which to store the parameters.
 * @return       ::SC_OK                       If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSensorExtrinsicParameters(ScDeviceHandle device, ScSensorExtrinsicParameters* pSensorExtrinsicParameters);

/**
 * @brief        Get the firmware version number.
 * @param[in]    device              The handle of the device on which to set the pulse count.
 * @param[out]   pFirmwareVersion    Pointer to a variable in which to store the returned fw value.
 * @param[in]    length              The maximum length is 64 bytes.
 * @return       ::SC_OK             If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFirmwareVersion(ScDeviceHandle device, char* pFirmwareVersion, int32_t length);

/**
 * @brief        Get the MAC from the device specified by <code>device</code>.
 * @param[in]    device         The handle of the device.
 * @param[out]   pMACAddress    Pointer to a buffer in which to store the device MAC address. the buffer default size is 18, and the last buffer set '\0'.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceMACAddress(ScDeviceHandle device, char* pMACAddress);

/**
 * @brief        Enables or disables DHCP. Default disabled
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetDeviceDHCPEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the device is in DHCP or not.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceDHCPEnabled(ScDeviceHandle device, bool* bEnabled);

/**
 * @brief        Set the IP address of the device in non-DHCP mode. The call takes effect after the device is restarted.
 * @param[in]    device         The handle of the device.
 * @param[in]    ipAddr         Pointer to a buffer in which to store the device IP address. the buffer default size is 16, and the last buffer set '\0'.
 * @param[in]    length         The length of the buffer.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetDeviceIPAddr(ScDeviceHandle device, const char* ipAddr, int32_t length);

/**
 * @brief        Get the IP address of the device in non-DHCP mode.
 * @param[in]    device         The handle of the device.
 * @param[out]   ipAddr         Pointer to a buffer in which to store the device IP address. the buffer default size is 16, and the last buffer set '\0'.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceIPAddr(ScDeviceHandle device, char* ipAddr);

/**
 * @brief        Set the subnet mask of the device in non-DHCP mode. The call takes effect after the device is restarted.
 * @param[in]    device         The handle of the device.
 * @param[in]    pMask          Pointer to a buffer in which to store the subnet mask address. the buffer default size is 16, and the last buffer set '\0'.
 * @param[in]    length         The length of the buffer.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetDeviceSubnetMask(ScDeviceHandle device, const char* pMask, int32_t length);

/**
 * @brief        Get the subnet mask of the device in non-DHCP mode.
 * @param[in]    device         The handle of the device.
 * @param[out]   pMask          Pointer to a buffer in which to store the device subnet mask address. the buffer default size is 16, and the last buffer set '\0'.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetDeviceSubnetMask(ScDeviceHandle device, char* pMask);

/**
 * @brief        Set the parameters for time sync, such as enable the NTP/PTP
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[in]    params       The parameters defined by ::ScTimeSyncConfig.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetRealTimeSyncConfig(ScDeviceHandle device, ScTimeSyncConfig params);

/**
 * @brief        Get the parameters for time sync,such as the status of the NTP/PTP
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   pParams      Pointer to a variable in which to store the returned value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetRealTimeSyncConfig(ScDeviceHandle device, ScTimeSyncConfig* pParams);

/**
 * @brief        Set the ToF frame rate.The interface takes a long time, about 500 ms.
 * @param[in]    device       The handle of the device on which to set the framerate.
 * @param[in]    value        The rate value. Different products have different maximum values. Please refer to the product specification.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetFrameRate(ScDeviceHandle device, int32_t value);

/**
 * @brief        Get the ToF frame rate.
 * @param[in]    device       The handle of the device on which to get the framerate.
 * @param[out]   pValue       The rate value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFrameRate(ScDeviceHandle device, int32_t* pValue);

/**
 * @brief        Set the working mode of the camera.
 * @param[in]    device      The handle of the device.
 * @param[in]    mode        The work mode of camera. For ActiveMode, set the Time filter default true, for SlaveMode, set the Time filter default false.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetWorkMode(ScDeviceHandle device, ScWorkMode mode);

/**
 * @brief        Get the working mode of the camera.
 * @param[in]    device      The handle of the device.
 * @param[out]   pMode       The work mode of camera.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetWorkMode(ScDeviceHandle device, ScWorkMode* pMode);

/**
 * @brief        Set the count of frame in SC_SOFTWARE_TRIGGER_MODE.
 *				 The more frames there are, the better frame's quality after algorithm processing
 * @param[in]    device       The handle of the device on which to set the parameter
 * @param[in]    frameCount	  The count of frame, in range [1,10].
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetSoftwareTriggerParameter(ScDeviceHandle device, uint8_t frameCount);

/**
 * @brief        Get the count of framer in SC_SOFTWARE_TRIGGER_MODE.
 * @param[in]    device       The handle of the device from which to get the parameter
 * @param[out]   pframeCount  Pointer to a variable in which to store the count of frame, in range [1,10].
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSoftwareTriggerParameter(ScDeviceHandle device, uint8_t* pframeCount);

/**
 * @brief        Get a frame in SC_SOFTWARE_TRIGGER_MODE.
 *               Call the scSetSoftwareTriggerParameter API to improve the quality of depth frame.
 * @param[in]    device      The handle of the device.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSoftwareTriggerOnce(ScDeviceHandle device);

/**
 * @brief        Set the input signal parameters for Hardware Trigger.
 * @param[in]    device       The handle of the device
 * @param[in]    params       Pointer to a variable in which to store the parameters.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetInputSignalParamsForHWTrigger(ScDeviceHandle device, ScInputSignalParamsForHWTrigger params);

/**
 * @brief        Get the Input signal parameters for Hardware Trigger.
 * @param[in]    device       The handle of the device
 * @param[out]   pParams      Pointer to a variable in which to store the returned value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetInputSignalParamsForHWTrigger(ScDeviceHandle device, ScInputSignalParamsForHWTrigger* pParams);

/**
 * @brief        Set the device GMM gain on a device.
 * @param[in]    device       The handle of the device on which to set the GMM gain.
 * @param[in]    gmmgain      The value of IRGMM Gain. Different products have different maximum value. Please refer to the product specification.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetIRGMMGain(ScDeviceHandle device, uint8_t gmmgain);

/**
 * @brief        Returns the the device's GMM gain.
 * @param[in]    device       The handle of the device from which to get the GMM gain.
 * @param[out]   pGmmgain     Pointer to a variable in which to store the returned GMM gain.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetIRGMMGain(ScDeviceHandle device, uint8_t* pGmmgain);

/**
 * @brief        Set the device IR GMM Correction on a device.
 * @param[in]    device       The handle of the device.
 * @param[in]    params       The value of IR GMM Correction.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetIRGMMCorrection(ScDeviceHandle device, const ScIRGMMCorrectionParams params);

/**
 * @brief        Return the device IR GMM Correction on a device.
 * @param[in]    device       The handle of the device.
 * @param[out]   params       The value of IR GMM Correction.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetIRGMMCorrection(ScDeviceHandle device, ScIRGMMCorrectionParams* pParams);

/**
 * @brief        Set the color image pixel format on the device specified by <code>device</code>. Currently only RGB and BGR formats are supported.
 * @param[in]    device         The handle of the device to set the pixel format.
 * @param[in]    pixelFormat    The color pixel format to use. Pass in one of the values defined by ::ScPixelFormat. Others cameras support only
 *                              <code>SC_PIXEL_FORMAT_RGB_888_JPEG</code> and <code>SC_PIXEL_FORMAT_BGR_888_JPEG</code>.
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetColorPixelFormat(ScDeviceHandle device, ScPixelFormat pixelFormat);

/**
 * @brief        Set the color Gain with the exposure mode of Color sensor in SC_EXPOSURE_CONTROL_MODE_MANUAL.
 * @param[in]    device       The handle of the device.
 * @param[in]    params       The value of color Gain.Different products have different maximum value. Please refer to the product specification.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetColorGain(ScDeviceHandle device, float params);

/**
 * @brief        Get the color Gain.
 * @param[in]    device       The handle of the device.
 * @param[out]   params       The value of color Gain.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetColorGain(ScDeviceHandle device, float* pParams);

/**
 * @brief        Get a list of image resolutions supported by Sensor
 * @param[in]    device       The handle of the device.
 * @param[in]    type         The sensor type
 * @param[out]   pList        List of supported resolutions
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSupportedResolutionList(ScDeviceHandle device, ScSensorType type, ScResolutionList* pList);

/**
 * @brief        Set the color frame Resolution.
 * @param[in]    device       The handle of the device.
 * @param[in]    w            The width of color image
 * @param[in]    h            The height of color image
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetColorResolution(ScDeviceHandle device, int32_t w, int32_t h);

/**
 * @brief        Returns the the color frame Resolution.
 * @param[in]    device       The handle of the device.
 * @param[out]   pW           The width of color image
 * @param[out]   pH           The height of color image
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetColorResolution(ScDeviceHandle device, int32_t* pW, int32_t* pH);

/**
 * @brief        Set the exposure mode of sensor.
 * @param[in]    device          The handle of the device on which to set the exposure control mode.
 * @param[in]    sensorType      The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[in]    exposureType    The exposure control mode.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetExposureControlMode(ScDeviceHandle device, ScSensorType sensorType, ScExposureControlMode controlMode);

/**
 * @brief        Get the exposure mode of sensor.
 * @param[in]    device           The handle of the device on which to get the exposure control mode.
 * @param[in]    sensorType       The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[out]   pControlMode     The exposure control mode.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetExposureControlMode(ScDeviceHandle device, ScSensorType sensorType, ScExposureControlMode* pControlMode);

/**
 * @brief        Set the exposure time of sensor.
 * @param[in]    device          The handle of the device on which to set the exposure time  in microseconds.
 * @param[in]    sensorType      The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[in]    exposureTime    The exposure time. The value must be within the maximum exposure time of sensor.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetExposureTime(ScDeviceHandle device, ScSensorType sensorType, int32_t exposureTime);

/**
 * @brief        Get the exposure time of sensor.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    sensorType       The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[out]   pExposureTime    The exposure time.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetExposureTime(ScDeviceHandle device, ScSensorType sensorType, int32_t* pExposureTime);

/**
 * @brief        Set the maximum exposure time of color sensor in automatic mode. The interface is used in automatic mode.
 * @param[in]    device          The handle of the device on which to set the exposure time in microseconds.
 * @param[in]    exposureTime    The exposure time. The value must be within the maximum exposure time of sensor.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetColorAECMaxExposureTime(ScDeviceHandle device, int32_t exposureTime);

/**
 * @brief        Get the maximum exposure time of color sensor in automatic mode. The interface is used in automatic mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[out]   pExposureTime    The exposure time.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetColorAECMaxExposureTime(ScDeviceHandle device, int32_t* pExposureTime);

/**
 * @brief        Get the maximum exposure time of sensor.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    sensorType       The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::ScSensorType.
 * @param[out]   pMaxExposureTime The maximum exposure time. The maximum exposure time is different at different frame rates.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetMaxExposureTime(ScDeviceHandle device, ScSensorType sensorType, int32_t* pMaxExposureTime);

/**
 * @brief        Enables or disables the HDR Mode of the ToF sensor with SC_EXPOSURE_CONTROL_MODE_MANUAL. Default enabled,
 *               so if you want switch to the SC_EXPOSURE_CONTROL_MODE_AUTO, set HDR Mode disable firstly.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetHDRModeEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the HDR Mode of ToF sensor feature is enabled or disabled.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetHDRModeEnabled(ScDeviceHandle device, bool* bEnabled);

/**
 * @brief        Get the count of frame in HDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[out]   pCount           The frame count.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFrameCountOfHDRMode(ScDeviceHandle device, int32_t* pCount);

/**
 * @brief        Set the exposure time of depth sensor with the frameIndex in HDR mode.
 * @param[in]    device          The handle of the device on which to set the exposure time  in microseconds.
 * @param[in]    frameIndex      The frameIndex from 0 to the count (get by scGetFrameCountOfHDRMode).
 * @param[in]    exposureTime    The exposure time. The value must be within the maximum exposure time of sensor.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetExposureTimeOfHDR(ScDeviceHandle device, uint8_t frameIndex, int32_t exposureTime);

/**
 * @brief        Get the exposure time of depth sensor with the frameIndex in HDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    frameIndex       The frameIndex from 0 to the count (get by scGetFrameCountOfHDRMode).
 * @param[out]   pExposureTime    The exposure time.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetExposureTimeOfHDR(ScDeviceHandle device, uint8_t frameIndex, int32_t* pExposureTime);

/**
 * @brief        Get the maximum exposure time of depth sensor with the frameIndex in HDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    frameIndex       The frameIndex from 0 to the count (get by scGetFrameCountOfHDRMode).
 * @param[out]   pMaxExposureTime The maximum exposure time. The maximum exposure time is different at different frame rates.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetMaxExposureTimeOfHDR(ScDeviceHandle device, uint8_t frameIndex, int32_t* pMaxExposureTime);

/**
 * @brief        Enables or disables the WDR Mode of the ToF sensor. Default enabled
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetWDRModeEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the WDRMode of ToF sensor feature is enabled or disabled.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetWDRModeEnabled(ScDeviceHandle device, bool* bEnabled);

/**
 * @brief        Get the count of frame in WDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[out]   pCount           The frame count.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFrameCountOfWDRMode(ScDeviceHandle device, int32_t* pCount);

/**
 * @brief        Set the exposure time of depth sensor with the frameIndex in WDR mode.
 * @param[in]    device          The handle of the device on which to set the exposure time  in microseconds.
 * @param[in]    frameIndex      The frameIndex from 0 to the count (get by scGetFrameCountOfWDRMode).
 * @param[in]    exposureTime    The exposure time. The value must be within the maximum exposure time of sensor.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetExposureTimeOfWDR(ScDeviceHandle device, uint8_t frameIndex, int32_t exposureTime);

/**
 * @brief        Get the exposure time of depth sensor with the frameIndex in WDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    frameIndex       The frameIndex from 0 to the count (get by scGetFrameCountOfWDRMode).
 * @param[out]   pExposureTime    The exposure time.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetExposureTimeOfWDR(ScDeviceHandle device, uint8_t frameIndex, int32_t* pExposureTime);

/**
 * @brief        Get the maximum exposure time of depth sensor with the frameIndex in WDR mode.
 * @param[in]    device           The handle of the device on which to get the exposure time in microseconds.
 * @param[in]    frameIndex       The frameIndex from 0 to the count (get by scGetFrameCountOfWDRMode).
 * @param[out]   pMaxExposureTime The maximum exposure time. The maximum exposure time is different at different frame rates.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetMaxExposureTimeOfWDR(ScDeviceHandle device, uint8_t frameIndex, int32_t* pMaxExposureTime);

/**
 * @brief        Set the parameters of the Time filter.
 * @param[in]    device       The handle of the device
 * @param[in]    params       Pointer to a variable in which to store the parameters.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetTimeFilterParams(ScDeviceHandle device, ScTimeFilterParams params);

/**
 * @brief        Get the parameters of the Time Filter feature.
 * @param[in]    device       The handle of the device
 * @param[out]   pParams      Pointer to a variable in which to store the returned value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetTimeFilterParams(ScDeviceHandle device, ScTimeFilterParams* pParams);

/**
 * @brief        Set the parameters of the Confidence filter.
 * @param[in]    device       The handle of the device
 * @param[in]    params       Pointer to a variable in which to store the parameters.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetConfidenceFilterParams(ScDeviceHandle device, ScConfidenceFilterParams params);

/**
 * @brief        Get the parameters of the ConfidenceFilter feature.
 * @param[in]    device       The handle of the device
 * @param[out]   pParams      Pointer to a variable in which to store the returned value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetConfidenceFilterParams(ScDeviceHandle device, ScConfidenceFilterParams* pParams);

/**
 * @brief        Set the parameters of the FlyingPixel filter.
 * @param[in]    device       The handle of the device.
 * @param[in]    params       Pointer to a variable in which to store the parameters.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetFlyingPixelFilterParams(ScDeviceHandle device, const ScFlyingPixelFilterParams params);

/**
 * @brief        Get the parameters of the FlyingPixel filter.
 * @param[in]    device       The handle of the device
 * @param[out]   pParams      Pointer to a variable in which to store the returned value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFlyingPixelFilterParams(ScDeviceHandle device, ScFlyingPixelFilterParams* params);

/**
 * @brief        Enables or disables the FillHole filter
 * @param[in]    device       The handle of the device.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetFillHoleFilterEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the FillHole Filter feature is enabled or disabled.
 * @param[in]    device       The handle of the device
 * @param[out]   pEnabled     Pointer to a variable in which to store the returned Boolean value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetFillHoleFilterEnabled(ScDeviceHandle device, bool* pEnabled);

/**
 * @brief        Enables or disables the Spatial filter
 * @param[in]    device       The handle of the device.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetSpatialFilterEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the Spatial Filter feature is enabled or disabled.
 * @param[in]    device       The handle of the device
 * @param[out]   pEnabled     Pointer to a variable in which to store the returned Boolean value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetSpatialFilterEnabled(ScDeviceHandle device, bool* pEnabled);

/**
 * @brief        Enables or disables transforms a color image into the geometry of the depth sensor. When enabled, scGetFrame() can\n
 *               be invoked passing ::ScTransformedColorFrame as the frame type for get a color image which each pixel matches the \n
 *               corresponding pixel coordinates of the depth sensor. The resolution of the transformed color frame is the same as that\n
 *               of the depth image.
 * @param[in]    device       The handle of the device on which to enable or disable mapping.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetTransformColorImgToDepthSensorEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the transformed of the color image to depth sensor space feature is enabled or disabled.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   bEnabled     Pointer to a variable in which to store the returned Boolean value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetTransformColorImgToDepthSensorEnabled(ScDeviceHandle device, bool* bEnabled);

/**
 * @brief        Enables or disables transforms the depth map into the geometry of the color sensor. When enabled, scGetFrame() can\n
 *               be invoked passing ::ScTransformedDepthFrame as the frame type for get a depth image which each pixel matches the \n
 *               corresponding pixel coordinates of the color sensor. The resolution of the transformed depth frame is the same as that\n
 *               of the color image.
 * @param[in]    device       The handle of the device on which to enable or disable mapping.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetTransformDepthImgToColorSensorEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Returns the Boolean value of whether the transformed of the depth image to color space feature is enabled or disabled.
 * @param[in]    device       The handle of the device on which to enable or disable the feature.
 * @param[out]   bEnabled     Pointer to a variable in which to store the returned Boolean value.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetTransformDepthImgToColorSensorEnabled(ScDeviceHandle device, bool* bEnabled);

/**
 * @brief        Returns the point value of the frame that the mapping of the depth image to Color space.
 * @param[in]    device           The handle of the device on which to enable or disable the feature.
 * @param[in]    depthPoint       The point in depth frame.
 * @param[in]    colorSize        The size(x = w,y = h) of color frame.
 * @param[out]   pPointInColor    The point in the color frame.
 * @return       ::SC_OK          If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scTransformDepthPointToColorPoint(const ScDeviceHandle device, const ScDepthVector3 depthPoint, const ScVector2u16 colorSize, ScVector2u16* pPointInColor);

/**
 * @brief        Converts the input points from depth coordinate space to world coordinate space.
 * @param[in]    device          The handle of the device on which to perform the operation.
 * @param[in]    pDepthVector    Pointer to a buffer containing the x, y, and z values of the depth coordinates to be converted. \n
 *                               x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
 *                               z is measured in millimeters, based on the ::ScPixelFormat depth frame.
 * @param[out]   pWorldVector    Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
 * @param[in]    pointCount      The number of points to convert.
 * @param[in]    pSensorParam    The intrinsic parameters for the depth sensor. See ::ScSensorIntrinsicParameters.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scConvertDepthToPointCloud(ScDeviceHandle device, ScDepthVector3* pDepthVector, ScVector3f* pWorldVector, int32_t pointCount, ScSensorIntrinsicParameters* pSensorParam);

/**
 * @brief        Converts the input Depth frame from depth coordinate space to world coordinate space on the device. Currently supported depth
 *               image types are SC_DEPTH_FRAME and SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME.
 * @param[in]    device         The handle of the device on which to perform the operation.
 * @param[in]    pDepthFrame    The depth frame.
 * @param[out]   pWorldVector   Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates,
 *                              measured in millimeters. The length of pWorldVector must is (ScFrame.width * ScFrame.height).
 * @return       ::SC_OK        If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scConvertDepthFrameToPointCloudVector(ScDeviceHandle device, const ScFrame* pDepthFrame, ScVector3f* pWorldVector);

/**
 * @brief        Set the parameters by Json file that can be saved by ScepterGUITool.
 * @param[in]    device       The handle of the device.
 * @param[in]    pfilePath    Pointer to the path of Json file.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetParamsByJson(ScDeviceHandle device, char* pfilePath);

/**
 * @brief        Export the parameter initialization file from the device.
 * @param[in]    device       The handle of the device.
 * @param[in]    pfilePath    Pointer to the path of parameter initialization file.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scExportParamInitFile(ScDeviceHandle device, char* pfilePath);

/**
 * @brief        Import the parameter initialization file into the device and take effect after reboot the device.
 * @param[in]    device       The handle of the device.
 * @param[in]    pfilePath    Pointer to the path of parameter initialization file.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scImportParamInitFile(ScDeviceHandle device, char* pfilePath);

/**
 * @brief        Restore the parameter initialization file of the device.
 * @param[in]    device       The handle of the device.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scRestoreParamInitFile(ScDeviceHandle device);

/**
 * @brief        Reboot the camera.
 * @param[in]    device          The handle of the device
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scRebootDevie(ScDeviceHandle device);

/**
 * @brief        Set hotplug status callback function.
 * @param[in]    pCallback    Pointer to the callback function. See ::PtrHotPlugStatusCallback
 * @param[in]    pUserData    Pointer to the user data. See ::PtrHotPlugStatusCallback
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scSetHotPlugStatusCallback(PtrHotPlugStatusCallback pCallback, const void* pUserData);

/**
 * @brief        Input the firmware file path and start upgrading device firmware.
 * @param[in]    device       The handle of the device.
 * @param[in]    pImgPath     Pointer to the path of firmware file. The firmware upgrade file is in .img format.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scStartUpgradeFirmWare(ScDeviceHandle device, char* pImgPath);

/**
 * @brief        Get firmware upgrade status and progress.
 * @param[in]    device       The handle of the device.
 * @param[out]   pStatus      Pointer to the status of firmware upgrade. 0 indicates normal, other values indicate anomalies.
 * @param[out]   pProcess     Pointer to the process of firmware upgrade, in range [0, 100]. Under normal circumstances, 100 indicates a successful upgrade.
 * @return       ::SC_OK      if the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scGetUpgradeStatus(ScDeviceHandle device, int32_t* pStatus, int32_t* pProcess);

#endif /* SCEPTER_API_H */
