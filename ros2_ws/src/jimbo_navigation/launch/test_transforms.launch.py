#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
    
    # TF test node
    tf_test_node = Node(
        package='jimbo_navigation',
        executable='tf_test',
        name='tf_test',
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [FindPackageShare('jimbo_navigation'), '/config/robot_view.rviz']],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_description_launch,
        tf_test_node,
        rviz_node
    ]) 