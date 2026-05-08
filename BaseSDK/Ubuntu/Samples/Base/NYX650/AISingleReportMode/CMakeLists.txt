project(AISingleReportMode)
add_executable( ${PROJECT_NAME} AISingleReportMode.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api)