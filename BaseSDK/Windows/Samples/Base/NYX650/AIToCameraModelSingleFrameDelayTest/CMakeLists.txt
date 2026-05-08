project(AIToCameraModelSingleFrameDelayTest)
add_executable( ${PROJECT_NAME} AIToCameraModelSingleFrameDelayTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api)