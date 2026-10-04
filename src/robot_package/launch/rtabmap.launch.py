"""Start the RTAB-Map SLAM node, fed directly by oak_camera."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    new_map = LaunchConfiguration('new_map')
    odom_topic = LaunchConfiguration('odom_topic')

    rtabmap_args = dict(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'database_path': LaunchConfiguration('database_path'),
            # The robot drives on flat ground: solve x, y and yaw only.
            'Reg/Force3DoF': 'true',
        }],
        # Straight from oak_camera: RGB and depth share a stamp, are the same
        # size, and depth is aligned to RGB.
        remappings=[
            ('rgb/image', 'camera/rgb/image_raw'),
            ('depth/image', 'camera/depth/image_raw'),
            ('rgb/camera_info', 'camera/rgb/camera_info'),
            ('odom', odom_topic),
        ],
    )
    # '-d' deletes the old database so the run starts a fresh map; without it
    # RTAB-Map keeps growing the map stored in database_path.
    rtabmap_new = Node(arguments=['-d'], condition=IfCondition(new_map),
                       **rtabmap_args)
    rtabmap_resume = Node(condition=UnlessCondition(new_map), **rtabmap_args)

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('odom_topic', default_value='odometry/filtered'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/.ros/rtabmap.db')),
        DeclareLaunchArgument(
            'new_map', default_value='true',
            description="Delete the old database at 'database_path' on start"),
        rtabmap_new,
        rtabmap_resume,
    ])
