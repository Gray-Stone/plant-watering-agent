from moveit_configs_utils import MoveItConfigsBuilder


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
    moveit_config = MoveItConfigsBuilder("stretch", package_name="stretch3_moveit").to_moveit_configs()

    stretch_mover_share = get_package_share_directory('stretch_mover')

    moveit_cmder = Node(
        package="stretch_mover",
        executable="moveit_cmder",
        parameters=[
            moveit_config.robot_description_kinematics
        ],
        output="screen"
    )

    return LaunchDescription([
        moveit_cmder,
    ])
