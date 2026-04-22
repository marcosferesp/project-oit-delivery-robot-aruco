cmake_minimum_required(VERSION 3.8)
project(aruco_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# PART A: Find the Tools
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# PART B: Include the Header Files
include_directories(include)

# PART C: Build the Executable
add_executable(aruco_detector_node 
  src/main.cpp 
  src/aruco_detector.cpp
)

# PART D: Glue it all together
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  image_transport
  OpenCV
)

# PART E: Install it
install(TARGETS
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()