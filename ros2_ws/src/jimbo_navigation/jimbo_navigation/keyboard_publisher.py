# keyboard_emergency_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.get_logger().info("Press 'e' to toggle Emergency Stop ON/OFF. Press 'q' to quit.")

        self.emergency = False
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'e':
                self.emergency = not self.emergency
                msg = Bool()
                msg.data = self.emergency
                self.pub.publish(msg)
                status = "ON" if self.emergency else "OFF"
                self.get_logger().info(f"Emergency Stop toggled: {status}")
            elif key == 'q':
                self.get_logger().info("Quitting keyboard emergency publisher.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
