project(DeviceStartStopStreaming)
add_executable( ${PROJECT_NAME} DeviceStartStopStreaming.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
