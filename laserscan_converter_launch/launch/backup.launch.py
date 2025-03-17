from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for scan topic'
        ),

        # PointCloud2 -> LaserScan 변환 노드
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),  # ✅ ZED의 PointCloud 입력
                ('scan', '/scanner/scan')  # ✅ 최종 LaserScan 출력
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.3,
                'max_height': 0.7,
                'angle_min': -1.5708,  # -90도
                'angle_max': 1.5708,  # 90도
                'angle_increment': 0.0087,  # 0.5도 간격
                'scan_time': 0.001,
                'range_min': 0.3,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'qos_overrides./cloud_in.subscriber.reliability': 'Best Effort',
                'publish_latched_scan': True  # ✅ Lazy subscription 방지
            }],
            name='pointcloud_to_laserscan'
        )
    ])
