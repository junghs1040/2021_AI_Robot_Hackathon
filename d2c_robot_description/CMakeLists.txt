cmake_minimum_required(VERSION 3.0.2)
project(d2c_robot_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
  tf
  urdf
  xacro
)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

 install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(kinematic src/kinematic.cpp src/IK_Solver.cpp)
add_dependencies(kinematic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic ${catkin_LIBRARIES})