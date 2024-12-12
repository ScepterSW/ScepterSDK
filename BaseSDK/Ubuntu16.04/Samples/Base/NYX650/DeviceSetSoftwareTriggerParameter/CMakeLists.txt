project(DeviceSetSoftwareTriggerParameter)
add_executable( ${PROJECT_NAME} DeviceSetSoftwareTriggerParameter.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
