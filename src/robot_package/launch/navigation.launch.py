"""Start Nav2 and the frontier explorer on top of robot.launch.py (camera, EKF, RTAB-Map)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_package'), 'config', 'nav2_params.yaml')

    # The costmaps' obstacle layers take point clouds, not depth images: turn
    # each depth pixel into a 3D point. Decimated and voxelized to spare the Pi.
    depth_points = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        output='screen',
        parameters=[{
            'decimation': 4,
            'voxel_size': 0.05,
            'max_depth': 4.0,
            'approx_sync': True,
        }],
        remappings=[
            ('depth/image', 'camera/depth/image_raw'),
            # Depth is aligned to RGB, so the RGB intrinsics apply
            ('depth/camera_info', 'camera/rgb/camera_info'),
            ('cloud', 'camera/depth/points'),
        ],
    )

    # Only the servers the default navigate_to_pose behavior tree needs.
    # Plain nodes rather than one composed container: easier to read logs and
    # restart one on its own.
    nav2_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    nav2 = [Node(package=package, executable=name, name=name, output='screen',
                 parameters=[params_file])
            for package, name in nav2_nodes]
    # Configures and activates the servers above, in order
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [name for _, name in nav2_nodes],
        }],
    )

    # Nav2's /cmd_vel -> per-wheel ticks/s on /motor_pid_cmd (needs
    # robot.launch.py's default motor_driver:=pid). Keep max_tps equal to
    # MAX_TARGET_TPS in pi_pico_PID/main.c: 3046 ticks/s is ~0.33 m/s per wheel,
    # which caps Nav2's speeds in nav2_params.yaml.
    cmd_vel = Node(
        package='robot_package',
        executable='cmd_vel',
        output='screen',
        parameters=[{
            'wheel_radius': 0.0335,
            'ticks_per_rev': 1960.0,
            'wheel_base': 0.21,
            'max_tps': 3046,
        }],
    )
    # Picks frontiers on /map and sends them to Nav2; waits for Nav2 to come up.
    # In `ros2 run robot_package keyboard_odom`, SPACE pauses it and takes over,
    # BACKSPACE hands control back.
    explore = Node(package='robot_package', executable='explore', output='screen')

    return LaunchDescription([
        depth_points,
        *nav2,
        lifecycle_manager,
        cmd_vel,
        explore,
    ])
