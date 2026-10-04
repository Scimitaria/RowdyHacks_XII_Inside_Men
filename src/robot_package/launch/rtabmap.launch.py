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
            # 2D occupancy grid on /map from the depth image, used by the
            # explore node to find unmapped space (-1 cells).
            'Grid/Sensor': '1',
            # Keep the grid 3D: /cloud_map is built from it, and with 'false'
            # it is flattened onto the floor. /map is still projected to 2D.
            'Grid/3D': 'true',
            # Mark space between the camera and obstacles as free, not just
            # floor the camera saw; otherwise seen space stays "unknown".
            'Grid/RayTracing': 'true',
            # 'Grid/RangeMax': '3.0',  # OAK depth gets noisy past a few metre
            'Grid/MaxObstacleHeight': '1.0',  # ignore ceilings and overhangs

            'RGBD/LinearUpdate': '0',
            'RGBD/AngularUpdate': '0',
            # Let a couple of "free" observations override an obstacle,
            # and cap how sure the map can get that a cell is occupied.
            'GridGlobal/ProbMiss': '0.2',
            'GridGlobal/ProbClampingMax': '0.8',
            # Longer rays clear more space behind where people stood.
            # Balance against OAK depth noise.
            'Grid/RangeMax': '4.0',
            
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
