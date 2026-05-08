#ifndef SCEPTER_MORPH_API_H
#define SCEPTER_MORPH_API_H

/**
 * @file Scepter_Morph_api.h
 * @brief Scepter Morph API header file.
 * Copyright (c) 2024 Goermicro Inc.
 */

/*! \mainpage Scepter Morph API Documentation
 *
 * \section intro_sec Introduction
 *
 * Welcome to the Scepter Morph API documentation. This documentation enables you to quickly get started in your
 * development efforts to programmatically interact with the Scepter Morph .
 */

#include "Scepter_define.h"

typedef enum
{
    AI_CONTINUOUS_RUN_MODE = 0x00,   //!< Enter the continuous mode.
    AI_SINGLE_RUN_MODE     = 0x01,   //!< Enter the single run mode, at this time need to invoke scAIModuleTriggerOnce, to trigger frame once and get the algo result of this frame.
    AI_SINGLE_REPORT_MODE  = 0x02    //!< Enter the single report mode. at this time need to invoke scAIModuleTriggerOnce, to trigger an algo result.
} ScAIModuleMode;

typedef enum
{
    AI_MODULE_TRANSFER_DOWNLOAD = 0x01,   //!< Download AI module to device.
    AI_MODULE_TRANSFER_UPLOAD   = 0x02    //!< Upload AI module from device.
} ScAIModuleTransferType;

#pragma pack(push, 1)

typedef struct
{
    uint32_t resultIndex;       //!< The index of the AI result.
    uint8_t* pResultData;       //!< A buffer containing the AI result's data.
    uint32_t dataLen;           //!< The length of AI result, in bytes.
    uint64_t resultTimestamp;   //!< The timestamp(in milliseconds) when the AI result be generated on the device AI mode.
} ScAIResult;

#pragma pack(pop)

/**
 * @brief        Set the AI module start or stop working.
 * @param[in]    device          The handle of the device on which to set the AI module state.
 * @param[in]    bEnabled        Set to <code>true</code> to start working or <code>false</code> to stop working.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleSetEnabled(ScDeviceHandle device, bool bEnabled);

/**
 * @brief        Get the AI module working state.
 * @param[in]    device          The handle of the device on which to get the AI module state.
 * @param[out]   pEnabled        Get to <code>true</code> to is working or <code>false</code> to is not working.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetEnabled(ScDeviceHandle device, bool* pEnabled);

/**
 * @brief        Set the param for the AI module
 * @param[in]    device          The handle of the device from which to set the param value.
 * @param[in]    paramID         The ID of the parameter.
 * @param[in]    pBuffer         Pointer to a buffer containing the property value.
 * @param[in]    bufferSize      The size, in bytes, of the parameter value contained in pBuffer. The max size is 65535.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleSetParam(ScDeviceHandle device, uint32_t paramID, void* pBuffer, uint16_t bufferSize);

/**
 * @brief        Get the param for the AI module
 * @param[in]    device          The handle of the device from which to get the param value.
 * @param[in]    paramID         The ID of the parameter.
 * @param[out]   pBuffer         Pointer to a buffer containing the property value.
 * @param[out]   pBufferSize     The size, in bytes, of the buffer.The max size is 65535.
 * @return       ::SC_OK         If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetParam(ScDeviceHandle device, uint32_t paramID, void** pBuffer, uint16_t* pBufferSize);

/**
 * @brief        Set the working mode of the AI module.
 * @param[in]    device      The handle of the device.
 * @param[in]    mode        The mode defined by ::ScAIModuleMode.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleSetWorkMode(ScDeviceHandle device, ScAIModuleMode mode);

/**
 * @brief        Gets the working mode of the AI module.
 * @param[in]    device      The handle of the device.
 * @param[out]   mode        The mode defined by ::ScAIModuleMode.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetWorkMode(ScDeviceHandle device, ScAIModuleMode* mode);

/**
 * @brief        Do trigger once if the AI module is in AI_SINGLE_RUN_MODE.
 * @param[in]    device      The handle of the device.
 * @return       ::SC_OK     If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleTriggerOnce(ScDeviceHandle device);

/**
 * @brief        Enables or disables the frame, true: do algorithm with the frameType
 * @param[in]    device       The handle of the device.
 * @param[in]    frameType    The type defined by ::ScFrameType.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleSetInputFrameTypeEnabled(ScDeviceHandle device, ScFrameType frameType, bool bEnabled);

/**
 * @brief        Get the frame is enables or disables , true: do AI mode with the frameType
 * @param[in]    device       The handle of the device.
 * @param[in]    frameType    The type defined by ::ScFrameType.
 * @param[out]   pEnabled     Get to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetInputFrameTypeEnabled(ScDeviceHandle device, ScFrameType frameType, bool* pEnabled);

/**
 * @brief        Enables or disables the frame, true: the frameType can be getted from the device
 * @param[in]    device       The handle of the device.
 * @param[in]    frameType    The type defined by ::ScFrameType.
 * @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleSetPreviewFrameTypeEnabled(ScDeviceHandle device, ScFrameType frameType, bool bEnabled);

/**
 * @brief        Get the frame is enables or disables , true: the frameType can be getted from the device
 * @param[in]    device       The handle of the device.
 * @param[in]    frameType    The type defined by ::ScFrameType.
 * @param[out]   pEnabled     Get to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetPreviewFrameTypeEnabled(ScDeviceHandle device, ScFrameType frameType, bool* pEnabled);

/**
 * @brief        Get the result of the AI mode
 * @param[in]    device       The handle of the device.
 * @param[in]    waitTime     The unit is millisecond, the value is in the range (0,65535).
 * @param[out]   pAIResult    Pointer to a buffer in which to store the signal on which AI result is ready to be get.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleGetResult(ScDeviceHandle device, uint16_t waitTime, ScAIResult* pAIResult);

/**
 * @brief        Upgrade AI mode file
 * @param[in]    device       The handle of the device.
 * @param[in]    pfilekPath   Pointer to a buffer containing the AI mode file path.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleTransfer(ScDeviceHandle device, ScAIModuleTransferType transferType, const char* pfilekPath, const char* parentDic);

/**
 * @brief        Get the status of upgrade
 * @param[in]    device       The handle of the device.
 * @param[out]   pState		  Pointer to a buffer containing the status of upgrade.
 * @param[out]   pProcess     Pointer to a buffer containing the process of upgrade. The value of process is in the range [1,100].
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleTransferStates(ScDeviceHandle device, uint8_t* pState, uint8_t* pProcess);

/**
 * @brief        Delete AI mode
 * @param[in]    device       The handle of the device.
 * @return       ::SC_OK      If the function succeeded, or one of the error values defined by ::ScStatus.
 */
SCEPTER_C_API_EXPORT ScStatus scAIModuleDelete(ScDeviceHandle device);
#endif /* SCEPTER_MORPH_API_H */
