# ekf_launch.py - Launch file for EKF Node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
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
# rosbag 재생
    rosbag = ExecuteProcess(            
            # cmd=['ros2', 'bag', 'play', '/home/twins/ros2_ws/rosbag2_2025_04_14-22_12_34', '--clock'],        
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_12_34', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_37_55', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_39_55', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_42_48', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_44_17', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_46_16', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_47_27', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_49_14', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_52_14', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_14/rosbag2_2025_04_14-22_53_59', '--clock'],
              
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_03_12', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_04_25', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_07_49', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_08_59', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_10_21', '--clock'],
            cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-00_11_29', '--clock'],
            # cmd=['ros2', 'bag', 'play', '/home/user/isaac_ros_ws/rosbag_4_16_imu/rosbag2_2025_04_16-07_43_58', '--clock'],

            output='screen'
        )

    imu_base_meas = Node(
        package="zed_bot",
        executable="imu_base_meas",
        parameters=[{'use_sim_time': True}],
        output="screen"
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

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}, ekf_config_path],
        # parameters=[ekf_config_path],
        remappings=[
            ('/odometry/filtered', '/odom_ekf'),
            ('/cmd_vel', '/rover_cmd_vel')
        ]
    )

    data_logger = Node(
        package="zed_bot",
        executable="data_logger",
        name="data_logger",
        output="screen",
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        rosbag,
        imu_base_meas,
        fwd_vio_base_meas,
        bwd_vio_base_meas,
        # data_logger,
        ekf_filter_node,
        rviz
    ])
