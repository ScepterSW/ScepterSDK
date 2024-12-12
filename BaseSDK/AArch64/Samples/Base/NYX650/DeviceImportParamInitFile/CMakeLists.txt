project(DeviceImportParamInitFile)
add_executable( ${PROJECT_NAME} DeviceImportParamInitFile.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
