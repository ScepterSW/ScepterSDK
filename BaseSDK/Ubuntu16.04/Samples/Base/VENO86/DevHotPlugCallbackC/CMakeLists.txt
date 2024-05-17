project(DevHotPlugCallbackC)
add_executable( ${PROJECT_NAME} DeviceHotPlugCallbackC.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
