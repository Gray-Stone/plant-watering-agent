cmake_minimum_required(VERSION 3.8)
project(stretch_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(trajectory_control src/trajectory_control.cpp)

ament_target_dependencies(
  trajectory_control
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2 
  tf2_ros
  trajectory_msgs
  )

install(TARGETS trajectory_control DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS  
  scripts/QuickTrajTest.py  
  DESTINATION lib/${PROJECT_NAME} )

# install(DIRECTORY
#   launch
#   meshes
#   config # urdf and srdf are under config
  
#   DESTINATION share/${PROJECT_NAME}/)



ament_package()
