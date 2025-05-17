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

    rover_controller_config_path = os.path.join(
        get_package_share_directory('zed_bot'),  # TODO: replace with your actual package name
        'config',
        'reg_rover_controller.config.yaml'
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

    rob_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[rob_state_pub_params],
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

    reg_rover_controller = Node(
        package="zed_bot",
        executable="reg_rover_controller",
        parameters=[rover_controller_config_path],
        output="screen",
    )


    return LaunchDescription(
        [            
            zed_wrapper_launch,            
            rob_state_pub,     
            imu_launch,
            imu_base_meas,        
            fwd_vio_base_meas,    
            bwd_vio_base_meas, 
            ekf_filter_node,
            reg_rover_controller,
            rviz 
        ]
    )