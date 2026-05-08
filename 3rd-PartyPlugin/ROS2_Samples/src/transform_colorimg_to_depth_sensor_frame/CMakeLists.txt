cmake_minimum_required(VERSION 3.5)
project(transform_colorimg_to_depth_sensor_frame)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(transform_colorimg_to_depth_sensor_frame src/transform_colorimg_to_depth_sensor_frame.cpp)

target_include_directories(transform_colorimg_to_depth_sensor_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(transform_colorimg_to_depth_sensor_frame PUBLIC ${OpenCV_INCLUDE_DIRS} $ENV{AMENT_PREFIX_PATH}/include/cv_bridge )

target_link_libraries(transform_colorimg_to_depth_sensor_frame ${OpenCV_LIBRARIES} )

ament_target_dependencies(
  transform_colorimg_to_depth_sensor_frame
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
)

install(TARGETS transform_colorimg_to_depth_sensor_frame
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
