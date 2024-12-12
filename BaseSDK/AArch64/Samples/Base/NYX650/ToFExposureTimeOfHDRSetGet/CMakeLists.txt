project(ToFExposureTimeOfHDRSetGet)
add_executable( ${PROJECT_NAME} ToFExposureTimeOfHDRSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
