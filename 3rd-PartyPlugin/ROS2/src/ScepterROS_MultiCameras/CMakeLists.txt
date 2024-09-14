cmake_minimum_required(VERSION 3.5)
set(CMAKE_INSTALL_RPATH "$ORIGIN/")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

project(ScepterROS_MultiCameras)

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

if($ENV{ROS_DISTRO} STREQUAL jazzy)
    ADD_DEFINITIONS(-DJAZZY)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED) 
find_package(PCL REQUIRED COMPONENTS common io)
#find_package(boost REQUIRED)

add_definitions(${PCL_DEFINITIONS})

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


find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_library(libScepter_api NAMES libScepter_api.so PATHS dependencies/Lib/)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include include dependencies/Include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} 
 /usr/include/eigen3/
 /usr/include/boost/
 $ENV{AMENT_PREFIX_PATH}/include/cv_bridge
 $ENV{AMENT_PREFIX_PATH}/include/image_transport
 $ENV{AMENT_PREFIX_PATH}/include/tf2_geometry_msgs)

 
 
LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS})
 
add_executable(scepter_multicameras src/scepter_driver.cpp
                                 src/scepter_manager.cpp
                                 include/scepter_manager.hpp)

file(GLOB PCL_LIBRARIES /usr/lib/x86_64-linux-gnu/libpcl_*)
                                 
target_link_libraries(scepter_multicameras  ${libScepter_api} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(scepter_multicameras rclcpp sensor_msgs std_msgs cv_bridge tf2 tf2_ros pcl_conversions)

install(
  DIRECTORY dependencies/Lib/ 
  DESTINATION lib/${PROJECT_NAME}  
)

install(TARGETS
  scepter_multicameras
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
