project(ToFExposureTimeOfWDRSetGet)
add_executable( ${PROJECT_NAME} ToFExposureTimeOfWDRSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
