import os
from glob import glob
from setuptools import setup

package_name = 'zed_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cchyun',
    maintainer_email='cchyun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'p_tuning_rover_controller = zed_bot.p_tuning_rover_controller:main',
            'theta_meas_rover_controller = zed_bot.theta_meas_rover_controller:main',
            'reg_rover_controller = zed_bot.reg_rover_controller:main',
            'rover_controller = zed_bot.rover_controller:main',
            'imu_base_meas = zed_bot.imu_base_meas:main',
            'vio_base_meas = zed_bot.vio_base_meas:main',
            'odom_publisher = zed_bot.odom_publisher:main',
            'data_logger = zed_bot.data_logger:main',
        ],
    },
)