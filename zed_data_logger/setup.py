from setuptools import find_packages, setup

package_name = 'zed_data_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zedbot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kon',
    maintainer_email='kon@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_odom = zed_data_logger.zed_odom:main',
            'zed_depth = zed_data_logger.zed_depth:main',
            'zed_camera = zed_data_logger.zed_camera:main',
            'subscribe = zed_data_logger.subscribe:main',
            'cmd_vel_publisher = zed_data_logger.cmd_vel_publisher:main'
        ],
    },
)
