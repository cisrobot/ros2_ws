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
    data_logger_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'data_logger.config.yaml'
    )
    
    data_logger = Node(
        package="zed_bot",
        executable="data_logger",
        parameters=[data_logger_config_path],
        output="screen",
    )

    return LaunchDescription(
        [            
            data_logger
        ]
    )