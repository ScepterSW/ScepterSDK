project(AIGetFrameAndResultSync)
add_executable( ${PROJECT_NAME} AIGetFrameAndResultSync.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess pthread)