import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    # ***** Launch arguments *****
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    zed_cartographer_prefix = get_package_share_directory('zed_cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(zed_cartographer_prefix, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='backpack_2d.lua'  # 🔹 Cartographer 설정 파일
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')

    return LaunchDescription([
        # ***** Declare Launch Arguments *****
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to Cartographer config directory'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for Cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if false'),

        # ***** Robot State Publisher (URDF) *****
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': LaunchConfiguration('robot_description'),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # ***** Cartographer Node *****
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('/points2', '/zed/zed_node/point_cloud/cloud_registered'),  # 🔹 ZED 포인트 클라우드 사용
                ('/odom', '/odometry/filtered'),
            ],
        ),

        # ***** Occupancy Grid (map 생성) *****
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }],
            output='screen'
        ),
    ])
