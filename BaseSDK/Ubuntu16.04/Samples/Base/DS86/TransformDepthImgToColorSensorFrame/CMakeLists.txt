project(TransformDepthImgToColorSensorFrame)
add_executable( ${PROJECT_NAME} TransformDepthImgToColorSensorFrame.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
