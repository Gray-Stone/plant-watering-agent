from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path

def generate_launch_description():

    stretch_mover_share = get_package_share_directory('stretch_mover')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_mover_share, "/launch/rtabmap_mapping.launch.py"]),
        # launch_arguments={"stretch_mode": "gamepad"}.items(),
        launch_arguments={"stretch_mode": "position"}.items(),
    )

    nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [stretch_mover_share, "/launch/nav2.launch.py"]))
    yolo_nodes_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [stretch_mover_share, "/launch/yolo_subsystem.py"]))

    return LaunchDescription([
        rtabmap_launch,
        # nav2_launch,
        yolo_nodes_launch,
    ])
