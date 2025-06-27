#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np

class TFTest(Node):
    def __init__(self):
        super().__init__('tf_test')
        
        # Initialize TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to check transforms
        self.timer = self.create_timer(1.0, self.check_transforms)
        
        self.get_logger().info('TF Test node initialized')
    
    def check_transforms(self):
        """Check if transforms are available"""
        try:
            # Check transform from base_link to laser_frame
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'laser_frame', self.get_clock().now())
            self.get_logger().info(f'Transform base_link -> laser_frame: {transform.transform.translation}')
            
            # Check transform from base_link to camera_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', self.get_clock().now())
            self.get_logger().info(f'Transform base_link -> camera_link: {transform.transform.translation}')
            
        except Exception as e:
            self.get_logger().warn(f'Transform not available: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 