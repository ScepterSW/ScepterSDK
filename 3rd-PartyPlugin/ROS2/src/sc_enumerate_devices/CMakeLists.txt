cmake_minimum_required(VERSION 3.5)
set(CMAKE_INSTALL_RPATH "$ORIGIN/")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
project(sc_enumerate_devices)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
    ADD_DEFINITIONS(-DJAZZY)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
    ADD_DEFINITIONS(-DHUMBLE)
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
    ADD_DEFINITIONS(-DFOXY)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_library(libScepter_api NAMES libScepter_api.so PATHS dependencies/Lib/)
include_directories(dependencies/Include)

add_executable(sc_enumerate_devices src/sc_enumerate_devices.cpp)

target_link_libraries(sc_enumerate_devices  ${libScepter_api})
ament_target_dependencies(sc_enumerate_devices rclcpp)

install(
  DIRECTORY dependencies/Lib/ 
  DESTINATION lib/${PROJECT_NAME}  
)

install(TARGETS
  sc_enumerate_devices
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
