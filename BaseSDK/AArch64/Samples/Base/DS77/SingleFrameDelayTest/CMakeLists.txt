project(SingleFrameDelayTest)
add_executable( ${PROJECT_NAME} SingleFrameDelayTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
