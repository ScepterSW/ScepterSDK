# Install script for directory: /work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DevHotPlugCallbackC/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DevHotPlugCallbackCpp/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceConnectBySN/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceConnectByIP/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceHWTriggerMode/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceInfoGet/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceParamSetGet/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceSearchAndConnect/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceSetFrameRate/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceSetParamsByJson/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceStartStopStreaming/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/DeviceSWTriggerMode/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/FrameCaptureAndSave/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnection/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/MultiConnectionInMultiThread/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudCaptureAndSave/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSave/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/ColorResolutionChange/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/ColorExposureTimeSetGet/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformColorImgToDepthSensorFrame/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/TransformDepthImgToColorSensorFrame/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/ToFExposureTimeSetGet/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/ToFFiltersSetGet/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudCaptureAndSaveDepthImgToColorSensor/cmake_install.cmake")
  include("/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/PointCloudVectorAndSaveDepthImgToColorSensor/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/DS77C/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
