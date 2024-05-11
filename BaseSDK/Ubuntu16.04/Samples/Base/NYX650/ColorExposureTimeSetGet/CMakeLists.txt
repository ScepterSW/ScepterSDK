project(ColorExposureTimeSetGet)
add_executable( ${PROJECT_NAME} ColorExposureTimeSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
