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

    yolo_process_params = {
        'input_topic': '/camera/color/image_raw',
        'aligned_depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
        'yolo_model': model_path,
        'yolo_result_topic': '/yolo_ros/detections',
        'result_image_topic': '/yolo_ros/result_image',
        'min_depth_range': 0.3,
        'max_depth_range': 3.0,
        "world_frame": "base_link",
        "debug": True,
        "verbose": True,
    }


    simple_depth_process = Node(
        package="stretch_mover",
        executable="simple_depth_process.py",
        parameters=[yolo_process_params],
        output='screen',
    )



    return LaunchDescription([
        simple_depth_process,
    ])
