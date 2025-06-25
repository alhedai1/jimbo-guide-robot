#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jimbo_navigation'),
            '/launch/robot_description.launch.py'
        ])
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'map_update_interval': 5.0,
            'max_queue_size': 10,
            'use_interactive_mode': False
        }],
        output='screen'
    )
    
    # Map server for saving maps
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True
        }],
        arguments=['--ros-args', '-p', 'save_map_timeout:=5.0'],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [FindPackageShare('jimbo_navigation'), '/config/slam_view.rviz']],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_description_launch,
        slam_toolbox_node,
        map_server_node,
        rviz_node
    ]) 