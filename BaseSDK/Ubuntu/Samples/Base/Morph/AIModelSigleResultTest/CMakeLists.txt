project(AIModelSigleResultTest)
add_executable( ${PROJECT_NAME} AIModelSigleResultTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api pthread)