project(DeviceSearchAndConnect)
add_executable( ${PROJECT_NAME} DeviceSearchAndConnect.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
