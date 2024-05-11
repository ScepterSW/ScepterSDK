project(DeviceInfoGet)
add_executable( ${PROJECT_NAME} DeviceInfoGet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
