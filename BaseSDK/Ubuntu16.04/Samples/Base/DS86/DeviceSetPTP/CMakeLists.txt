project(DeviceSetPTP)
add_executable( ${PROJECT_NAME} DeviceSetPTP.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
