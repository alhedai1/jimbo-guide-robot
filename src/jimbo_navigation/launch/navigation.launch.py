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
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=[FindPackageShare('jimbo_navigation'), '/maps/my_map.yaml'],
        description='Path to map file'
    )
    
    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('jimbo_navigation'),
            '/launch/robot_description.launch.py'
        ])
    )
    
    # Navigation2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': [FindPackageShare('jimbo_navigation'), '/config/nav2_params.yaml']
        }.items()
    )
    
    # RViz2 for navigation visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [FindPackageShare('jimbo_navigation'), '/config/nav_view.rviz']],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        robot_description_launch,
        nav2_bringup_launch,
        rviz_node
    ]) 