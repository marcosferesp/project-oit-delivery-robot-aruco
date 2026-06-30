cmake_minimum_required(VERSION 3.8)
project(aruco_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################################
# PART 1 : Find the Tools #########
###################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

###################################
# PART 2 : Include the Header Files
###################################

include_directories(include)

###################################
# PART 3 : Build the Executables ##
###################################

# ArucoDetectorNode
add_executable(aruco_detector_node 
  src/main.cpp 
  src/aruco_detector.cpp
)

#MoverNode #ArucoFollowerNode
add_executable(mover 
  src/mover.cpp
)

# DynamicTerminalInputNode
add_executable(dispatcher
  src/dispatcher.cpp
)

###################################
# PART 4 : Glue them all together #
###################################

# ArucoDetectorNode
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  image_transport
  OpenCV
  std_msgs
)

#MoverNode #ArucoFollowerNode
ament_target_dependencies(mover
  rclcpp
  geometry_msgs
  std_msgs
)

# DynamicTerminalInputNode
ament_target_dependencies(dispatcher
  rclcpp
  std_msgs
)

###################################
# PART 5 : Install everything #####
###################################

target_link_libraries(dispatcher readline)

install(TARGETS
  aruco_detector_node
  mover
  dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()