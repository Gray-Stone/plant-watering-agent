cmake_minimum_required(VERSION 3.8)
project(stretch_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(fmt)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# needed by depth processing
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloDetection.msg"
  "msg/YoloDetectionList.msg"
  "msg/KnownObject.msg"
  "msg/KnownObjectList.msg"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)


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

add_executable(masked_depth_process src/masked_depth_process.cpp)
ament_target_dependencies(masked_depth_process
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  vision_msgs
  visualization_msgs
  cv_bridge
  image_geometry
  message_filters
  pcl_ros
  tf2_eigen
  tf2_ros
  OpenCV
  PCL
)

add_executable(moveit_cmder src/moveit_cmder.cpp)
ament_target_dependencies(moveit_cmder
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  vision_msgs
  visualization_msgs
  tf2_eigen
  tf2_ros
  octomap_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(moveit_cmder fmt::fmt)

add_executable(move_interface_test src/move_interface_test.cpp)
ament_target_dependencies(move_interface_test
  rclcpp
  moveit_ros_planning_interface
)
target_link_libraries(moveit_cmder fmt::fmt)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(masked_depth_process "${cpp_typesupport_target}")
target_link_libraries(moveit_cmder "${cpp_typesupport_target}")

ament_python_install_package(stretch_mover_utils)

install(TARGETS trajectory_control DESTINATION lib/${PROJECT_NAME})
install(TARGETS masked_depth_process DESTINATION lib/${PROJECT_NAME})
install(TARGETS moveit_cmder DESTINATION lib/${PROJECT_NAME})
install(TARGETS move_interface_test DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS  
  scripts/QuickTrajTest.py
  scripts/yolo_process.py
  scripts/simple_depth_process.py
  scripts/StatefulController.py
  DESTINATION lib/${PROJECT_NAME} )

install(DIRECTORY
  launch
  config
  models  
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
