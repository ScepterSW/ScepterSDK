project(AISingleRunMode)
add_executable( ${PROJECT_NAME} AISingleRunMode.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)