#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import subprocess
from launch.actions import TimerAction

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
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )

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
            'pointcloud.enable': 'true',
            'enable_rgbd': 'true',
            # 'depth_width': '640',
            # 'depth_height': '480',
            # 'depth_fps': '30'
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_realsense'))
    )
    
    # lidar launch 
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ])
    )
    

    # Motor interface node
    motor_node = Node(
        package='motor_interface_pkg',
        executable='motor_serial_node',
        name='motor_serial_node',
        # parameters=[{
        #     'port': '/dev/motor_usb',
        #     'baudrate': 115200,
        #     'wheel_radius': 0.0635,
        #     'wheel_base': 0.3
        # }],
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

    emergency_stop_node = Node(
            package='jimbo_navigation',
            executable='emergency_stop',
            name='emergency_stop',
            output='screen'
        )

    tracking_controller_node = Node(
            package='jimbo_navigation',
            executable='tracking_controller',
            name='tracking_controller',
            output='screen'
        )

    # delayed_tracking_node = TimerAction(
    #     period=5.0,
    #     actions=[tracking_controller_node]
    # )

    occupancy_node = Node(
        package='jimbo_navigation',
        executable='occ_grid',
        name='occ_grid',
        # parameters=[{
        #     'grid_size': 10.0,
        #     'resolution': 0.1
        # }],
        output='screen'
    )
    
    bso_hfc_node = TimerAction(
        period=1.0,  # seconds
        actions=[
            Node(
                package='jimbo_navigation',
                executable='bso_hfc',
                name='bso_hfc',
                # parameters=[{
                #     'target_distance': 1.0,
                #     'max_linear_velocity': 0.5,
                #     'max_angular_velocity': 1.0,
                # }],
                output='screen',
            )
        ]
    )

    dwa_node = TimerAction(
        period=1.0,  # seconds
        actions=[
            Node(
                package='jimbo_navigation',
                executable='dwa_controller',
                name='dwa_controller',
                output='screen'
            )
        ]
    )

    # mpc_node = TimerAction(
    #     period=1.0,  # seconds
    #     actions=[
    #         Node(
    #             package='jimbo_navigation',
    #             executable='mpc_follower',
    #             name='mpc_follower',
    #             output='screen',
    #         )
    #     ]
    # )

    # custom nav2 launch (no map_server, amcl)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jimbo_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('jimbo_navigation'),
                'config',
                'nav2_params.yaml')
        }.items()
    )
    
    # full nav2 stack
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

    # runs in separate terminal (press 'e' to toggle emergency stop, press 'q' to kill node)
    keyboard_publisher_node = Node(
        package='jimbo_navigation',
        executable='keyboard_publisher',
        name='keyboard_publisher',
        output='screen',
        prefix='gnome-terminal --'
    )

    urdf_path = '/home/jimbo/jimbo-guide-robot/ros2_ws/src/jimbo_navigation/urdf/jimbo_robot.urdf'
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_motor_arg,
        enable_rviz_arg,
        # enable_realsense_arg,
        robot_description_launch,
        rviz_node,
        motor_node,
        uwb_launch,
        lidar_launch,
        keyboard_publisher_node,
        # realsense_launch,
        emergency_stop_node,
        tracking_controller_node,
        nav_launch,
    ])
