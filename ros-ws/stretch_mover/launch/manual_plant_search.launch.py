from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path

def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    stretch_mover_share = get_package_share_directory('stretch_mover')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_mover_share, "/launch/rtabmap_mapping.launch.py"]),
        launch_arguments={"stretch_mode": "gamepad"}.items(),
        # launch_arguments={"stretch_mode": "navigation"}.items(),
    )

    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())


    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': LaunchConfiguration('stretch_mode') , 'broadcast_odom_tf': 'True'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))


    nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [stretch_mover_share, "/launch/nav2.launch.py"]))
    yolo_nodes_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [stretch_mover_share, "/launch/yolo_subsystem.py"]))

    return LaunchDescription([
        rtabmap_launch,
        nav2_launch,
        yolo_nodes_launch,
        # Robot driver.
        stretch_driver_launch,
        d435i_launch,
        # Teleop
        teleop_type_param,
        base_teleop_launch,

    ])
