project(CameraModelSigleFrameDelayTest)
add_executable( ${PROJECT_NAME} CameraModelSigleFrameDelayTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api pthread)