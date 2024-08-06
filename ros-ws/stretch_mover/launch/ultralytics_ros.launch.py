from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    stretch_mover_share = get_package_share_directory('stretch_mover')

    model_path = str(Path(stretch_mover_share) / "models/seg-water-pot-new.pt")

    ultralytics_ros_share = get_package_share_directory('ultralytics_ros')

    track_with_depth = Node(
        package="ultralytics_ros",
        executable="tracker_depth_node.py",
        parameters=[{
            # Param for yolo tracker
            'yolo_model': model_path,
            'input_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            
            'result_topic': 'yolo_ros/detect_result',
            'result_image_topic': 'yolo_ros/result_image',
            "conf_thres": 0.25,  # default 0.25

            # tracker_with_cloud param
            'camera_info_topic': '/camera/color/camera_info',
            # The point cloud topic. Actually d435i high res launch file didn't turn on point cloud
            # The default topic is /camera/depth/color/points.
            # We'll use rtabmap's local point cloud as quick subs.
            'lidar_topic': '/camera/depth/color/points',
            # 'lidar_topic': '/local_grid_obstacle',
            'yolo_result_topic':
                'yolo_ros/detect_result',  # point cloud side chain on output of 2d track
            'yolo_3d_result_topic': 'yolo_ros/detect_3d_box',
            'cluster_tolerance': 0.01,
            'voxel_leaf_size': 0.05,  # Copy default of rtabmap
            'min_cluster_size': 10,  # default 100, might leave out small pots
            'max_cluster_size': 10000,  # default 10000 
        }],
        output='screen',
    )

    # track_with_cloud = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         [ultralytics_ros_share, '/launch/tracker_with_cloud.launch.xml']),
    #     launch_arguments={
    #         # Param for yolo tracker
    #         'yolo_model': model_path,
    #         'input_topic': '/camera/color/image_raw',
    #         'result_topic': 'yolo_ros/detect_result',
    #         'result_image_topic': 'yolo_ros/result_image',
    #         "conf_thres": "0.25", # default 0.25

    #         # tracker_with_cloud param
    #         'camera_info_topic': '/camera/color/camera_info',
    #         # The point cloud topic. Actually d435i high res launch file didn't turn on point cloud
    #         # The default topic is /camera/depth/color/points.
    #         # We'll use rtabmap's local point cloud as quick subs.
    #         'lidar_topic': '/camera/depth/color/points',
    #         # 'lidar_topic': '/local_grid_obstacle',
    #         'yolo_result_topic':
    #             'yolo_ros/detect_result',  # point cloud side chain on output of 2d track
    #         'yolo_3d_result_topic': 'yolo_ros/detect_3d_box',
    #         'cluster_tolerance': "0.01",
    #         'voxel_leaf_size': "0.05",  # Copy default of rtabmap
    #         'min_cluster_size': "10",  # default 100, might leave out small pots
    #         'max_cluster_size': "10000",  # default 10000
    #     }.items(),
    # )
    return LaunchDescription([track_with_depth])
