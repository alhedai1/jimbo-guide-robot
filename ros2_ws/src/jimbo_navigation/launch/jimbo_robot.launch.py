#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
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
    
    enable_realsense_arg = DeclareLaunchArgument(
        'enable_realsense',
        default_value='false',
        description='Enable RealSense camera'
    )
    
    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor',
        default_value='true',
        description='Enable motor control (disable for simulation)'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='false',
        description='Enable safety monitor'
    )
    
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='Enable user following behavior'
    )
    
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable autonomous navigation'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # map_file_arg = DeclareLaunchArgument(
    #     'map',
    #     default_value=[FindPackageShare('jimbo_navigation'), '/maps/generated_map.yaml'],
    #     description='Path to map file (for navigation)'
    # )

    # nav2_params_file = LaunchConfiguration('params_file')

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=[FindPackageShare('jimbo_navigation'), '/config/nav2_params.yaml'],
        description='Path to Navigation2 parameters file'
    )
    
    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jimbo_navigation'),
                'launch',
                'robot_description.launch.py'
            ])
        ])
    )
    

    # Motor interface node (conditional) - start first
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
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_motor'))
    )

    # UWB launch (always enabled)
    uwb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uwb_interface'),
                'launch',
                'uwb.launch.py'
            ])
        ])
    )

    # Add a delay before starting user_follower to ensure motor node is ready
    from launch.actions import TimerAction
    follower_node = TimerAction(
        period=3.0,  # seconds
        actions=[
            Node(
                package='jimbo_navigation',
                executable='user_follower',
                name='user_follower',
                parameters=[{
                    'target_distance': 1.0,
                    'max_linear_velocity': 0.5,
                    'max_angular_velocity': 1.0,
                }],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_follower'))
            )
        ]
    )
    
    # RealSense camera launch (conditional)
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
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_realsense'))
    )
    
    # YDLidar launch (always enabled)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ])
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jimbo_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ])
    )
    
    # Launches full nav2 stack
    full_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': '/home/jimbo/jimbo-guide-robot/ros2_ws/src/jimbo_navigation/maps/map.yaml',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                FindPackageShare('jimbo_navigation'),
                'config',
                'nav2_params_full.yaml'
            ])
        }.items()
    )
    
    
    # RViz2 for visualization (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [FindPackageShare('jimbo_navigation'), '/config/config.rviz']],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        # enable_realsense_arg,
        enable_motor_arg,
        # # enable_safety_arg,
        enable_follower_arg,
        # enable_navigation_arg,
        enable_rviz_arg,
        # map_file_arg,
        # nav2_params_file_arg,
        robot_description_launch,
        rviz_node,
        motor_node,
        follower_node,
        # realsense_launch,
        lidar_launch,
        uwb_launch,
        # nav_launch
        # full_nav_launch
    ])