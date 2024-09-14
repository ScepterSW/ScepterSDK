cmake_minimum_required(VERSION 2.8)
project(DS77C_OpenCVSample)

set(CMAKE_SKIP_RPATH FALSE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_SKIP_INSTALL_RPATH_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/:../../../Lib:../Lib:../Samples/OpenCV/Thirdparty/opencv-3.4.1/lib:../Thirdparty/opencv-3.4.1/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions("-Wall -fPIC -D Linux")


include_directories("../../../Include" "../Thirdparty/opencv-3.4.1/include")
LINK_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../Lib/")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../PrecompiledSamples")


add_executable(${PROJECT_NAME} "FrameViewer.cpp")
target_link_libraries(${PROJECT_NAME} DSImgPreProcess Scepter_api opencv_core opencv_highgui opencv_imgproc pthread)
#set( CMAKE_BUILD_TYPE Debug )
