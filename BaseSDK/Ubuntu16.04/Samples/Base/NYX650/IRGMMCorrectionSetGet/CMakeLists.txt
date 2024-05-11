project(IRGMMCorrectionSetGet)
add_executable( ${PROJECT_NAME} IRGMMCorrectionSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Scepter_api DSImgPreProcess)
