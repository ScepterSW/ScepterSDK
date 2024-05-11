project(DeviceConnectByIP)
add_executable( ${PROJECT_NAME} DeviceConnectByIP.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
