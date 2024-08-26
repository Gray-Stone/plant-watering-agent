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
        launch_arguments={"use_rviz": "false"}.items(),
    )

    rviz_config = DeclareLaunchArgument('rviz_config',
                                        default_value=str(Path(stretch_mover_share) / 'config/All-combined.rviz'))
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    stretch_mode = DeclareLaunchArgument(
        'stretch_mode',
        default_value='gamepad', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        # default_value='trajectory', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='The mode in which the ROS driver commands the robot'
    )

    teleop_type_param = DeclareLaunchArgument(
        'teleop_type',
        default_value="joystick",
        description="how to teleop ('keyboard', 'joystick' or 'none')")

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={
            'mode': LaunchConfiguration('stretch_mode'),
            'broadcast_odom_tf': 'True'
        }.items())

    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [stretch_core_path, '/launch/d435i_high_resolution.launch.py']))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_mover_share, "/launch/nav2.launch.py"]))

    # These are things that needs to use yolo.

    # This one have way too much false positive
    model_path = str(Path(stretch_mover_share) / "models/cont_10k_water_pot_500.pt")
    model_path = str(Path(stretch_mover_share) / "models/fresh_water_pot_500.pt")

    yolo_params = {
        'input_topic': '/camera/color/image_raw',
        'aligned_depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        "conf_thres": 0.45,
        'yolo_model': model_path,
        'yolo_result_topic': '/yolo_ros/detections',
        'result_image_topic': '/camera/color/yolo_result_image',
        'min_depth_range': 0.3,
        'max_depth_range': 3.0,
        "world_frame": "map",

        # "debug": True,
        # "verbose": True,
    }

    yolo_process = Node(
        package="stretch_mover",
        executable="yolo_process.py",
        parameters=[yolo_params],
        output='screen',
    )
    simple_depth_process = Node(
        package="stretch_mover",
        executable="simple_depth_process.py",
        parameters=[yolo_params],
        output='screen',
    )

    stateful_controller = Node(
        package="stretch_mover",
        executable="StatefulController.py",
        parameters=[yolo_params],
        output='screen',
    )

    rviz_launch = Node(package='rviz2', executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        respawn=True,
        arguments=['-d', LaunchConfiguration('rviz_config')],
        )

    return LaunchDescription([
        # Params
        rviz_config,
        rviz_param,
        stretch_mode,

        # Robot driver.
        stretch_driver_launch,
        d435i_launch,
        # Teleop
        teleop_type_param,
        base_teleop_launch,
        rviz_launch,
        # Nav related
        rtabmap_launch,
        nav2_launch,

        # YOLO
        yolo_process,
        simple_depth_process,
        # stateful_controller,
    ])
