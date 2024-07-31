
## provided by stretch_driver

when running `ros2 launch stretch_core stretch_driver.launch.py mode:=gamepad broadcast_odom_tf:=True` These topics are available 

```
/battery
/diagnostics
/gamepad_joy
/imu_mobile_base
/imu_wrist
/is_gamepad_dongle
/is_homed
/is_runstopped
/joint_limits
/joint_states
/magnetometer_mobile_base
/mode
/odom
/parameter_events
/robot_description
/rosout
/stretch/cmd_vel
/stretch/joint_states
/stretch_gamepad_state
/tf
/tf_static
/tool
```

These service (not including parameter service)

```
/runstop
/self_collision_avoidance
/stop_the_robot
/stow_the_robot
/switch_to_gamepad_mode
/switch_to_navigation_mode
/switch_to_position_mode
/switch_to_trajectory_mode
```

## Provided by rtabmap

When running the `stretch_rtabmap/visual_mapping.launch.py` following nodes exists 

```
/camera
/d435i_accel_correction_node
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_state_publisher
/joint_trajectory_action
/joy_node
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/robot_state_publisher
/rqt_gui_py_node_103508
/rtabmap
/rviz
/stretch_driver
/teleop_twist_joy_node
/transform_listener_impl_5858bf9d8b70
/transform_listener_impl_58ca61b85ad0
```


This is what rtabmap provides 
```
ros2 node info /rtabmap          
/rtabmap
  Subscribers:
    /camera/aligned_depth_to_color/image_raw: sensor_msgs/msg/Image
    /camera/color/camera_info: sensor_msgs/msg/CameraInfo
    /camera/color/image_raw: sensor_msgs/msg/Image
    /global_pose: geometry_msgs/msg/PoseWithCovarianceStamped
    /goal: geometry_msgs/msg/PoseStamped
    /goal_node: rtabmap_msgs/msg/Goal
    /gps/fix: sensor_msgs/msg/NavSatFix
    /imu: sensor_msgs/msg/Imu
    /initialpose: geometry_msgs/msg/PoseWithCovarianceStamped
    /landmark_detection: rtabmap_msgs/msg/LandmarkDetection
    /landmark_detections: rtabmap_msgs/msg/LandmarkDetections
    /odom: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /republish_node_data: std_msgs/msg/Int32MultiArray
    /user_data_async: rtabmap_msgs/msg/UserData
  Publishers:
    /cloud_ground: sensor_msgs/msg/PointCloud2
    /cloud_map: sensor_msgs/msg/PointCloud2
    /cloud_obstacles: sensor_msgs/msg/PointCloud2
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /elevation_map: grid_map_msgs/msg/GridMap
    /global_path: nav_msgs/msg/Path
    /global_path_nodes: rtabmap_msgs/msg/Path
    /goal_out: geometry_msgs/msg/PoseStamped
    /goal_reached: std_msgs/msg/Bool
    /grid_prob_map: nav_msgs/msg/OccupancyGrid
    /info: rtabmap_msgs/msg/Info
    /labels: visualization_msgs/msg/MarkerArray
    /landmarks: geometry_msgs/msg/PoseArray
    /local_grid_empty: sensor_msgs/msg/PointCloud2
    /local_grid_ground: sensor_msgs/msg/PointCloud2
    /local_grid_obstacle: sensor_msgs/msg/PointCloud2
    /local_path: nav_msgs/msg/Path
    /local_path_nodes: rtabmap_msgs/msg/Path
    /localization_pose: geometry_msgs/msg/PoseWithCovarianceStamped
    /map: nav_msgs/msg/OccupancyGrid
    /mapData: rtabmap_msgs/msg/MapData
    /mapGraph: rtabmap_msgs/msg/MapGraph
    /mapOdomCache: rtabmap_msgs/msg/MapGraph
    /mapPath: nav_msgs/msg/Path
    /octomap_binary: octomap_msgs/msg/Octomap
    /octomap_empty_space: sensor_msgs/msg/PointCloud2
    /octomap_full: octomap_msgs/msg/Octomap
    /octomap_global_frontier_space: sensor_msgs/msg/PointCloud2
    /octomap_grid: nav_msgs/msg/OccupancyGrid
    /octomap_ground: sensor_msgs/msg/PointCloud2
    /octomap_obstacles: sensor_msgs/msg/PointCloud2
    /octomap_occupied_space: sensor_msgs/msg/PointCloud2
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /tf: tf2_msgs/msg/TFMessage
  Service Servers:
    /add_link: rtabmap_msgs/srv/AddLink
    /backup: std_srvs/srv/Empty
    /cancel_goal: std_srvs/srv/Empty
    /cleanup_local_grids: rtabmap_msgs/srv/CleanupLocalGrids
    /detect_more_loop_closures: rtabmap_msgs/srv/DetectMoreLoopClosures
    /get_map: nav_msgs/srv/GetMap
    /get_map_data: rtabmap_msgs/srv/GetMap
    /get_map_data2: rtabmap_msgs/srv/GetMap2
    /get_node_data: rtabmap_msgs/srv/GetNodeData
    /get_nodes_in_radius: rtabmap_msgs/srv/GetNodesInRadius
    /get_plan: nav_msgs/srv/GetPlan
    /get_plan_nodes: rtabmap_msgs/srv/GetPlan
    /get_prob_map: nav_msgs/srv/GetMap
    /global_bundle_adjustment: rtabmap_msgs/srv/GlobalBundleAdjustment
    /list_labels: rtabmap_msgs/srv/ListLabels
    /load_database: rtabmap_msgs/srv/LoadDatabase
    /log_debug: std_srvs/srv/Empty
    /log_error: std_srvs/srv/Empty
    /log_info: std_srvs/srv/Empty
    /log_warning: std_srvs/srv/Empty
    /octomap_binary: octomap_msgs/srv/GetOctomap
    /octomap_full: octomap_msgs/srv/GetOctomap
    /pause: std_srvs/srv/Empty
    /publish_map: rtabmap_msgs/srv/PublishMap
    /remove_label: rtabmap_msgs/srv/RemoveLabel
    /reset: std_srvs/srv/Empty
    /resume: std_srvs/srv/Empty
    /rtabmap/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /rtabmap/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /rtabmap/get_parameters: rcl_interfaces/srv/GetParameters
    /rtabmap/list_parameters: rcl_interfaces/srv/ListParameters
    /rtabmap/set_parameters: rcl_interfaces/srv/SetParameters
    /rtabmap/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /set_goal: rtabmap_msgs/srv/SetGoal
    /set_label: rtabmap_msgs/srv/SetLabel
    /set_mode_localization: std_srvs/srv/Empty
    /set_mode_mapping: std_srvs/srv/Empty
    /trigger_new_map: std_srvs/srv/Empty
    /update_parameters: std_srvs/srv/Empty
  Service Clients:
    /rtabmap/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /rtabmap/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /rtabmap/get_parameters: rcl_interfaces/srv/GetParameters
    /rtabmap/list_parameters: rcl_interfaces/srv/ListParameters
    /rtabmap/set_parameters: rcl_interfaces/srv/SetParameters
    /rtabmap/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Action Servers:

  Action Clients:
```

This is what node camera provides 
```
╰─➤  ros2 node info /camera                      
/camera
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /camera/accel/imu_info: realsense2_camera_msgs/msg/IMUInfo
    /camera/accel/metadata: realsense2_camera_msgs/msg/Metadata
    /camera/accel/sample: sensor_msgs/msg/Imu
    /camera/aligned_depth_to_color/camera_info: sensor_msgs/msg/CameraInfo
    /camera/aligned_depth_to_color/image_raw: sensor_msgs/msg/Image
    /camera/aligned_depth_to_color/image_raw/compressed: sensor_msgs/msg/CompressedImage
    /camera/aligned_depth_to_color/image_raw/compressedDepth: sensor_msgs/msg/CompressedImage
    /camera/aligned_depth_to_color/image_raw/theora: theora_image_transport/msg/Packet
    /camera/color/camera_info: sensor_msgs/msg/CameraInfo
    /camera/color/image_raw: sensor_msgs/msg/Image
    /camera/color/image_raw/compressed: sensor_msgs/msg/CompressedImage
    /camera/color/image_raw/compressedDepth: sensor_msgs/msg/CompressedImage
    /camera/color/image_raw/theora: theora_image_transport/msg/Packet
    /camera/color/metadata: realsense2_camera_msgs/msg/Metadata
    /camera/depth/camera_info: sensor_msgs/msg/CameraInfo
    /camera/depth/color/points: sensor_msgs/msg/PointCloud2
    /camera/depth/image_rect_raw: sensor_msgs/msg/Image
    /camera/depth/image_rect_raw/compressed: sensor_msgs/msg/CompressedImage
    /camera/depth/image_rect_raw/compressedDepth: sensor_msgs/msg/CompressedImage
    /camera/depth/image_rect_raw/theora: theora_image_transport/msg/Packet
    /camera/depth/metadata: realsense2_camera_msgs/msg/Metadata
    /camera/extrinsics/depth_to_accel: realsense2_camera_msgs/msg/Extrinsics
    /camera/extrinsics/depth_to_color: realsense2_camera_msgs/msg/Extrinsics
    /camera/extrinsics/depth_to_depth: realsense2_camera_msgs/msg/Extrinsics
    /camera/extrinsics/depth_to_gyro: realsense2_camera_msgs/msg/Extrinsics
    /camera/gyro/imu_info: realsense2_camera_msgs/msg/IMUInfo
    /camera/gyro/metadata: realsense2_camera_msgs/msg/Metadata
    /camera/gyro/sample: sensor_msgs/msg/Imu
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /tf_static: tf2_msgs/msg/TFMessage
  Service Servers:
    /camera/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /camera/device_info: realsense2_camera_msgs/srv/DeviceInfo
    /camera/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /camera/get_parameters: rcl_interfaces/srv/GetParameters
    /camera/list_parameters: rcl_interfaces/srv/ListParameters
    /camera/set_parameters: rcl_interfaces/srv/SetParameters
    /camera/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```