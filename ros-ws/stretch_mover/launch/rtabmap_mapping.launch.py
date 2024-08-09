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
    stretch_mover_share = get_package_share_directory('stretch_mover')
    stretch_rtabmap_path = get_package_share_directory('stretch_rtabmap')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    driver_mode_arg = DeclareLaunchArgument(
        'stretch_mode',
        default_value='position', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='The mode in which the ROS driver commands the robot'
    )

    rviz_config = DeclareLaunchArgument('rviz_config',
                                        default_value=str(Path(stretch_mover_share) / 'config/rtabmap.rviz'))

    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': LaunchConfiguration('stretch_mode') , 'broadcast_odom_tf': 'True'}.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/d435i_high_resolution.launch.py']))

    rtabmap_parameters = {

        # There might be a slight performance issue.
        # "wait_for_transform": 0.2,

        # Angular update to 0 doesn't actually keep updating it, linear update to 0 will do the trick
        "RGBD/LinearUpdate": '0.00',
        "RGBD/AngularUpdate": '0.1',

        "RGBD/CreateOccupancyGrid": 'True',
        # Haven't see other diff robot using this.
        "Odom/Holonomic": 'False',
        "Grid/RangeMax": '4.0',
        
        # rough value from the stored mode.
        "Grid/FootprintLength": "0.6",
        "Grid/FootprintWidth": "0.3",
        "Grid/FootprintHeight": "0.6",

        "Grid/MaxObstacleHeight": '2.0',
        "Grid/MaxGroundHeight": '0.02',
        "Grid/RayTracing": 'True',
        # "Grid/CellSize": 0.05 # default 0.05
    }
    rtabmap_mapping_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        arguments=['-d'],
        parameters=[
            rtabmap_parameters,
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            # TODO consider change this remap
            ('grid_map', 'map'),
        ],
        output='screen',
        )
    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    rviz_launch = Node(package='rviz2', executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        respawn=True,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        )

    return LaunchDescription([
        rviz_param,
        rviz_config,
        driver_mode_arg,
        teleop_type_param,
        stretch_driver_launch,
        d435i_launch,
        rtabmap_mapping_node,
        base_teleop_launch,
        rviz_launch,
    ])
