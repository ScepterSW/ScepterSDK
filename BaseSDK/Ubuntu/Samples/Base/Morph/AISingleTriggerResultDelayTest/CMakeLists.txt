project(AISingleTriggerResultDelayTest)
add_executable( ${PROJECT_NAME} AISingleTriggerResultDelayTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api pthread)