project(MultiConnection)
add_executable( ${PROJECT_NAME} MultiConnection.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
