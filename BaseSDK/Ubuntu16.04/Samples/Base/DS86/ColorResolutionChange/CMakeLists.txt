project(ColorResolutionChange)
add_executable( ${PROJECT_NAME} ColorResolutionChange.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
