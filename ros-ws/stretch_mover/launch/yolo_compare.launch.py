# This launch file is used for comparing multiple yolo models.
# Still expects the robot camera to be running.


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

    models :list[Path] = [
        # This is the older one, without turning on rotation augmentation
        Path(stretch_mover_share) / "models/seg-water-pot-new.pt",
        # 3 new sets of model, with rotation settings.
        Path(stretch_mover_share) / "models/lamb-watering-pot-2.pt",
        Path(stretch_mover_share) / "models/lamb-10kpot_cont-watering-area-2.pt",
        # pot only base, for comparison
        Path(stretch_mover_share) / "models/lamb_10kpot.pt",
    ]

    yolo_params = {
        'input_topic': '/camera/color/image_raw',
        'aligned_depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        "conf_thres": 0.5,
        'yolo_result_topic': '/yolo_ros/detections',
        'result_image_topic': 'yolo_result_image',

        'min_depth_range': 0.3,
        'max_depth_range': 3.0,
        "world_frame": "map",

        # "debug": True,
        # "verbose": True,
    }

    nodes = []
    for m in models:
        ns = m.stem
        nodes.append(
            Node(package="stretch_mover",
                 executable="yolo_process.py",
                 parameters=[yolo_params, {
                     'yolo_model': str(m)
                 }],
                 output='screen',
                 namespace=ns,
                 remappings=[('/camera/color/image_raw', ns + '/camera/color/image_raw')]))


    return LaunchDescription(nodes)
