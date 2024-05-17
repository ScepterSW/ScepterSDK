project(TransformColorImgToDepthSensorFrame)
add_executable( ${PROJECT_NAME} TransformColorImgToDepthSensorFrame.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
