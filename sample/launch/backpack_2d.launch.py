import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    # ***** File paths *****
    pkg_share = FindPackageShare('zed_cartographer').find('zed_cartographer')
    urdf_file = os.path.join(FindPackageShare('zed_bot').find('zed_bot'), 'urdf', 'robot_1.xacro')

    # URDF 파일을 Xacro로 변환하여 로봇 상태 퍼블리셔에 전달
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=Command(['xacro ', urdf_file]),
        description='Robot URDF description generated from Xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )


    # ***** Cartographer 노드 (여기서 tuple 문제 수정!) *****
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', '/home/twins/ros2_ws/src/zed_cartographer/config',  # 수정됨
            '-configuration_basename', 'backpack_2d.lua'],
        remappings=[
            ('points2', '/zed/zed_node/point_cloud/cloud_registered'),  # ZED 카메라의 포인트 클라우드 사용
            ('odom', '/odometry/filtered'),
        ],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}, {'resolution': 0.05}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_description_arg,
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
