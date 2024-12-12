# Install script for directory: /home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650

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
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DevHotPlugCallbackC/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DevHotPlugCallbackCpp/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceConnectBySN/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceConnectByIP/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceHWTriggerMode/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceInfoGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceParamSetGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceSearchAndConnect/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceSetFrameRate/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceSetParamsByJson/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceStartStopStreaming/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/DeviceSWTriggerMode/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/FrameCaptureAndSave/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/MultiConnection/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/MultiConnectionInMultiThread/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/PointCloudCaptureAndSave/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/PointCloudVectorAndSave/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/ColorResolutionChange/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/ColorExposureTimeSetGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/TransformColorImgToDepthSensorFrame/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/TransformDepthImgToColorSensorFrame/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/ToFExposureTimeSetGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/ToFFiltersSetGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/IRGMMCorrectionSetGet/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/PointCloudCaptureAndSaveDepthImgToColorSensor/cmake_install.cmake")
  include("/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/PointCloudVectorAndSaveDepthImgToColorSensor/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/peter/work/ScepterSDK/BaseSDK/Ubuntu18.04/Samples/Base/NYX650/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
