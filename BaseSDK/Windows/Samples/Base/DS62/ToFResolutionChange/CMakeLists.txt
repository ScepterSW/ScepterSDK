project(ToFResolutionChange)
add_executable( ${PROJECT_NAME} ToFResolutionChange.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api)