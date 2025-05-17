import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory("zed_bot"))
    xacro_file = os.path.join(pkg_path, "urdf", "rover.xacro")
    robot_description = xacro.process_file(xacro_file)
    rob_state_pub_params = {"robot_description": robot_description.toxml()}

    rover_controller_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'rover_controller.config.yaml'
    )
    ps5_joystick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ps5_joy'),
                'launch',
                'ps5_joy_teleop.launch.py'
            )
        ])
    )


    rob_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[rob_state_pub_params],
    )


    rover_controller = Node(
        package="zed_bot",
        executable="rover_controller",
        parameters=[rover_controller_config_path],
        output="screen",
    )


    return LaunchDescription(
        [            
            # ps5_joystick_cmd,
            rob_state_pub,
            rover_controller            
        ]
    )