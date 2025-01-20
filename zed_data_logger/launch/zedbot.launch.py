from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_data_logger',  # 패키지 이름
            executable='zed_odom',  # 실행 파일 이름
            name='zed_odom_node',  # 노드 이름
            output='screen'
        ),
        Node(
            package='zed_data_logger',
            executable='subscribe',
            name='subscribe_node',
            output='screen'
        ),
        Node(
            package='zed_data_logger',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher_node',
            output='screen'
        ),
    ])
