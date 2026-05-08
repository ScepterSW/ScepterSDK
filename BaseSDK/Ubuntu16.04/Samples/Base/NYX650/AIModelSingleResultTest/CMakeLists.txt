project(AIModelSingleResultTest)
add_executable( ${PROJECT_NAME} AIModelSingleResultTest.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api pthread)