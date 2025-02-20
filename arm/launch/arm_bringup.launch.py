from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import xacro

def generate_launch_description():
    arm_path = get_package_share_path('arm')

    urdf_file = os.path.join(arm_path, "urdf", "arm.urdf.xacro")
    robot_urdf =  xacro.process_file(urdf_file)
    arm_description = {'robot_description': robot_urdf.toxml()}

    control_params_path = os.path.join(
        arm_path,
        'params',
        'arm_control_params.yaml'
    )

    arm_control_node = Node(
        package='arm',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen',
        parameters=[control_params_path]
    )

    arm_send_cmd_params_path = os.path.join(
        arm_path,
        'params',
        'arm_send_cmd_params.yaml'
    )

    arm_send_cmd_node = Node(
        package='arm',
        executable='arm_send_cmd_node',
        name='arm_send_cmd_node',
        output='screen',
        parameters=[arm_send_cmd_params_path]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_state_publisher',
        output='screen',
        parameters=[arm_description],
        # namespace='/arm_state_publisher',
        remappings=[
            ('/joint_states', '/joint_arm_states')
        ]
    )


    return LaunchDescription([        
        robot_state_publisher_node,
        arm_control_node,       
        arm_send_cmd_node 
    ])
