cmake_minimum_required(VERSION 3.0.2)
project(d2c_robot_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  d2c_robot_msgs
  message_generation
)

#add_message_files(
#  FILES
#  D2cRobot.msg
#)

#generate_messages(
#  DEPENDENCIES 
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(d2c_robot_keyboard src/d2c_robot_keyboard.cpp)
target_link_libraries(d2c_robot_keyboard ${catkin_LIBRARIES})
