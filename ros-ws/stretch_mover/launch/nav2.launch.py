"""  
This launch file launches 

stretch_core/launch/rplidar.launch.py
stretch_nav2/launch/navigation_launch.py

with param from stretch_mover/config/

This is designed to be launch along side with rtabmap with depth camera (point cloud). 
Also depends on stretch_driver be already running or launch by other launch files.

"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    stretch_nav2_dir = get_package_share_directory('stretch_nav2')
    stretch_mover_share = get_package_share_directory('stretch_mover')

    stretch_core_path = get_package_share_directory('stretch_core')

    declare_nav2_param = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(stretch_mover_share, 'config', 'nav2_config_along_rtabmap.yaml'),
        description='Full path to the ROS2 parameters file for nav2 nodes')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    nav2_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [stretch_nav2_dir, '/launch/navigation_launch.py']),
                                            launch_arguments={
                                                'namespace': '',
                                                'params_file': nav2_params_file,
                                                'use_lifecycle_mgr': 'true',
                                                'map_subscribe_transient_local': 'true'
                                            }.items())
    return LaunchDescription([
        declare_nav2_param,
        rplidar_launch,
        nav2_include,
    ])
