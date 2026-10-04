"""Compress the camera images so they can be viewed from another computer.

Run this next to whatever launch starts oak_camera (e.g. robot.launch.py). It
publishes, alongside the raw topics (which are only sent if something
subscribes to them):
  /camera/rgb/image_raw/compressed         JPEG
  /camera/depth/image_raw/compressedDepth  PNG (16-bit depth)

Disable depth with `depth:=false` to save CPU and bandwidth.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def republish(name, topic, out_transport, condition=None):
    # republish reads the raw 'in' topic and publishes 'out/<out_transport>'.
    # Remapping the base name 'out' has no effect, so remap the full topic.
    return Node(
        package='image_transport',
        executable='republish',
        name=name,
        output='screen',
        parameters=[{'in_transport': 'raw', 'out_transport': out_transport}],
        remappings=[('in', topic),
                    (f'out/{out_transport}', f'{topic}/{out_transport}')],
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='true'),
        republish('rgb_republisher', 'camera/rgb/image_raw', 'compressed'),
        republish('depth_republisher', 'camera/depth/image_raw', 'compressedDepth',
                  condition=IfCondition(LaunchConfiguration('depth'))),
    ])
