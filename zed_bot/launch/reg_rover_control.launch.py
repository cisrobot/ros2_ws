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
    rover_controller_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'reg_rover_controller.config.yaml'
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

    rover_controller = Node(
        package="zed_bot",
        executable="reg_rover_controller",
        name="reg_rover_controller",
        parameters=[rover_controller_config_path],
        output="screen",
    )


    return LaunchDescription(
        [            
            # ps5_joystick_cmd,
            rover_controller            
        ]
    )