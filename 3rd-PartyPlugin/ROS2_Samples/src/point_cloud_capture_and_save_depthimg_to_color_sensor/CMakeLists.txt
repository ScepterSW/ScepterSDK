cmake_minimum_required(VERSION 3.5)
project(point_cloud_capture_and_save_depthimg_to_color_sensor)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) 
find_package(PCL REQUIRED)

add_executable(point_cloud_capture_and_save_depthimg_to_color_sensor src/point_cloud_capture_and_save_depthimg_to_color_sensor.cpp)

target_include_directories(point_cloud_capture_and_save_depthimg_to_color_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(point_cloud_capture_and_save_depthimg_to_color_sensor PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(point_cloud_capture_and_save_depthimg_to_color_sensor ${PCL_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

ament_target_dependencies(
  point_cloud_capture_and_save_depthimg_to_color_sensor
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
)

install(TARGETS point_cloud_capture_and_save_depthimg_to_color_sensor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
