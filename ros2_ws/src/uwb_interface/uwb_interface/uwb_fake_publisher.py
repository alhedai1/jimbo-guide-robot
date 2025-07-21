# uwb_fake_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import time

class FakeTargetPublisher(Node):
    def __init__(self):
        super().__init__('fake_target_pub')
        self.pub = self.create_publisher(PointStamped, '/uwb_filtered_position', 10)
        self.timer = self.create_timer(0.1, self.publish_target)
        self.t = 0.0

    def publish_target(self):
        msg = PointStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Move in a circle of radius 1m
        msg.point.x = 1.5 * np.cos(self.t)
        msg.point.y = 1.5 * np.sin(self.t)
        msg.point.z = 0.0

        self.t += 0.1
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
