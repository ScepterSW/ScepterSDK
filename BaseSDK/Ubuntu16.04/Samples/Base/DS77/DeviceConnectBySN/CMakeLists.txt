project(DeviceConnectBySN)
add_executable( ${PROJECT_NAME} DeviceConnectBySN.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
