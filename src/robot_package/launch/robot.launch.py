"""Start the whole stack: camera + IMU, wheel odometry, motor driver, JPEG image relay, EKF and RTAB-Map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_package'), 'launch')

    def include(name, condition=None, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_arguments.items(),
            condition=condition)

    # Publishes camera/rgb, camera/depth, and the IMU heading on imu/data
    # (which the EKF reads).
    # Keep the robot still for the first second while the gyro calibrates.
    oak_camera = Node(package='robot_package', executable='oak_camera',
                      output='screen')
    # Publishes /wheel/odom from the Pico's encoders (the EKF owns odom -> base_link).
    wheel_odom = Node(package='robot_package', executable='odom',
                      output='screen')
    # Sends /motor_cmd to the Pico. Drive with `ros2 run robot_package keyboard`
    # or `explore` in another terminal; it stops if they go quiet.
    motor = Node(package='robot_package', executable='motor',
                 output='screen')
    # Republishes camera/rgb as JPEG on camera/rgb/image_relay/compressed, for
    # viewing over Tailscale with `camera_viewer` or RViz.
    image_relay = Node(package='robot_package', executable='image_relay',
                       output='screen')

    # Where the camera sits on the robot. The rotation turns the robot frame
    # (x forward, y left, z up) into the camera's image frame (z forward,
    # x right, y down), which is what RTAB-Map expects.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', LaunchConfiguration('cam_x'),
                   '--y', LaunchConfiguration('cam_y'),
                   '--z', LaunchConfiguration('cam_z'),
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'oak_camera_optical'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('cam_x', default_value='0.0',
                              description='Camera forward offset from base_link (m)'),
        DeclareLaunchArgument('cam_y', default_value='0.1016',
                              description='Camera left offset from base_link (m), 4 in'),
        DeclareLaunchArgument('cam_z', default_value='0.3302',
                              description='Camera height above base_link (m), 13 in'),
        DeclareLaunchArgument('mapping', default_value='true',
                              description='Also run RTAB-Map'),
        DeclareLaunchArgument('new_map', default_value='true',
                              description='Delete the old RTAB-Map database on start'),
        oak_camera,
        wheel_odom,
        motor,
        image_relay,
        camera_tf,
        include('localization.launch.py'),
        include('rtabmap.launch.py', condition=IfCondition(LaunchConfiguration('mapping')),
                new_map=LaunchConfiguration('new_map')),
    ])
