project(DeviceIPAndSubnetMaskSet)
add_executable( ${PROJECT_NAME} DeviceIPAndSubnetMaskSet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
