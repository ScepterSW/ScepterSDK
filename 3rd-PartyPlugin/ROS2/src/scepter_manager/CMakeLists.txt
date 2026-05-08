cmake_minimum_required(VERSION 3.5)
set(CMAKE_INSTALL_RPATH "$ORIGIN/")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
project(scepter_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-format -Wno-pedantic -Wno-dev)
endif()

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    ADD_DEFINITIONS(-DJAZZY)
    if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
    endif()
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
    ADD_DEFINITIONS(-DHUMBLE)
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
    ADD_DEFINITIONS(-DFOXY)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_definitions(${PCL_DEFINITIONS})

find_library(libScepter_api NAMES libScepter_api.so PATHS dependencies/Lib/)
include_directories(include dependencies/Include ${OpenCV_INCLUDE_DIRS})

ament_auto_add_library(scepter_manager SHARED src/scepter_manager.cpp include/scepter_manager.hpp)

target_link_libraries(scepter_manager  ${libScepter_api})

rclcpp_components_register_node(scepter_manager
        PLUGIN "ScepterManager"
        EXECUTABLE scepter_manager_node_exe)

install(
  DIRECTORY dependencies/Lib/ 
  DESTINATION lib
)

install(TARGETS
  scepter_manager
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
