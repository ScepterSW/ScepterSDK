project(DeviceSetNTP)
add_executable( ${PROJECT_NAME} DeviceSetNTP.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
