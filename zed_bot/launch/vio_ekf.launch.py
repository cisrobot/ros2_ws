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
    ekf_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'ekf.yaml'
    )
    fwd_vio_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'fwd_vio_base.config.yaml'
    )
    bwd_vio_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'bwd_vio_base.config.yaml'
    )

    # arm 패키지의 arm_bringup.launch.py 경로 설정
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('arm'),
                'launch',
                'arm_bringup.launch.py'
            )
        ])
    )

    # zed_wrapper 패키지의 custom_zed_multi_camera.launch.py 경로 설정
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'custom_zed_multi_camera.launch.py'
            )
        ])
    )

    rover_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_bot'),
                'launch',
                'rover_control.launch.py'
            )
        ])
    )

    fwd_vio_base_meas = Node(
        package="zed_bot",
        executable="vio_base_meas",
        name="fwd_vio_base_meas",
        output="screen",
        parameters=[fwd_vio_config_path],
    )

    bwd_vio_base_meas = Node(
        package="zed_bot",
        executable="vio_base_meas",
        name="bwd_vio_base_meas",
        output="screen",
        parameters=[bwd_vio_config_path],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/odometry/filtered', '/odom_ekf'),
            ('/cmd_vel', '/rover_cmd_vel')
        ]
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('xsens_mti_ros2_driver'),
                'launch',
                'xsens_mti_node.launch.py'
            )
        ])
    )

    imu_base_meas = Node(
        package="zed_bot",
        executable="imu_base_meas",
        output="screen"
    )


    return LaunchDescription(
        [            
            zed_wrapper_launch,            
            imu_launch,
            imu_base_meas,        
            fwd_vio_base_meas,    
            bwd_vio_base_meas, 
            rover_control_launch,
            ekf_filter_node,
            rviz 
        ]
    )