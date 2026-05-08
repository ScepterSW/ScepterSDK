project(SafetyCameraGetResult)
add_executable( ${PROJECT_NAME} SafetyCameraGetResult.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api)