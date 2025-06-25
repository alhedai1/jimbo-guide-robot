#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('jimbo_navigation')
    
    # Get the path to the URDF file
    urdf_file = os.path.join(get_package_share_directory('jimbo_navigation'), 'urdf', 'jimbo_robot.urdf')
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description_config = file.read()
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Robot state publisher node (we'll use a simple URDF for now)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint state publisher node (for visualization)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Static transform publisher for RealSense camera
    # This connects the camera_link from URDF to the RealSense camera frame
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '0', '0', '0',  # x, y, z translation
            '0', '0', '0',  # roll, pitch, yaw rotation
            'camera_link',   # parent frame
            'camera_depth_frame'  # child frame (RealSense frame)
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        camera_tf_node
    ]) 