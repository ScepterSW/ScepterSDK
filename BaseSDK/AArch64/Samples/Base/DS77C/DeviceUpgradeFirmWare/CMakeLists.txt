project(DeviceUpgradeFirmWare)
add_executable( ${PROJECT_NAME} DeviceUpgradeFirmWare.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
