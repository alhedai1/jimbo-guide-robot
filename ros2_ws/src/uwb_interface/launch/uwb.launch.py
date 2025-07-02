from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwb_interface',
            executable='uwb_interface_node',
            name='uwb_interface_node',
            parameters=[{
                'tag_ports': ['/dev/uwb_front_left', '/dev/uwb_front_right', '/dev/uwb_back_left', '/dev/uwb_back_right'],
                'baudrate': 115200
            }],
            output='screen'
        )
    ])
