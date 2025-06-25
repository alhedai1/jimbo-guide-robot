#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('jimbo_navigation')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Motor interface node
    motor_node = Node(
        package='motor_interface_pkg',
        executable='motor_serial_node',
        name='motor_serial_node',
        parameters=[{
            'port': '/dev/motor_arduino',
            'baudrate': 115200,
            'wheel_radius': 0.0635,
            'wheel_base': 0.3
        }],
        output='screen'
    )
    
    # Safety monitor node
    safety_node = Node(
        package='jimbo_navigation',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'emergency_stop_distance': 0.3,
            'slow_down_distance': 0.5,
            'warning_distance': 0.8
        }],
        output='screen'
    )
    
    # User follower node
    follower_node = Node(
        package='jimbo_navigation',
        executable='user_follower',
        name='user_follower',
        parameters=[{
            'target_distance': 1.0,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'person_detection_threshold': 0.3
        }],
        output='screen'
    )
    
    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_depth': 'true',
            'enable_pointcloud': 'true',
            'enable_rgb': 'true',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30'
        }.items()
    )
    
    # YDLidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ])
    )
    
    # TF remapping for safety monitor
    safety_node_remapped = Node(
        package='jimbo_navigation',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'emergency_stop_distance': 0.3,
            'slow_down_distance': 0.5,
            'warning_distance': 0.8
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel_raw'),
            ('cmd_vel_filtered', 'cmd_vel')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        motor_node,
        safety_node_remapped,
        follower_node,
        realsense_launch,
        lidar_launch
    ]) 