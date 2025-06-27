from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_interface_pkg',
            executable='motor_serial_node',  # console_scripts 이름
            name='motor_serial_node',
            output='screen',
            emulate_tty=True,  # 로그 출력 안정화
            parameters=[{
                'port': '/dev/motor_arduino',
                'baudrate': 115200,
                'wheel_radius': 0.05,
                'wheel_base': 0.3
            }]
        ),
    ])
