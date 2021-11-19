cmake_minimum_required(VERSION 3.0.2)
project(d2c_robot_libs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  d2c_robot_msgs
  darknet_ros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES d2c_robot_libs
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(robot_node
  src/robot_node.cpp
  src/d2c_control.cpp
  src/serving_command.cpp
  
)

add_dependencies(robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_node ${catkin_LIBRARIES} )
