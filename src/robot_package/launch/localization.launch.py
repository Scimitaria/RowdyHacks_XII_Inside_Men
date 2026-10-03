"""Run the IMU adapter and the robot_localization EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('robot_package'), 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_package',
            executable='imu_adapter',
            name='imu_adapter',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
