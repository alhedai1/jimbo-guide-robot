#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import struct

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters
        self.declare_parameter('emergency_stop_distance', 0.3)  # 30cm
        self.declare_parameter('slow_down_distance', 0.5)       # 50cm
        self.declare_parameter('warning_distance', 0.8)         # 80cm
        
        self.emergency_stop_dist = self.get_parameter('emergency_stop_distance').value
        self.slow_down_dist = self.get_parameter('slow_down_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        
        # State variables
        self.lidar_data = None
        self.depth_data = None
        self.emergency_stop = False
        self.slow_down = False
        self.warning = False
        
        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.filtered_cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_filtered', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.depth_sub = self.create_subscription(PointCloud2, 'camera/depth/points', self.depth_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz
        
        self.get_logger().info('Safety Monitor initialized')
    
    def lidar_callback(self, msg):
        """Process lidar data for obstacle detection"""
        self.lidar_data = msg
        
        # Check for obstacles in front of robot
        # Define front cone angle (e.g., ±30 degrees from forward direction)
        front_cone_angle = np.radians(30)  # 30 degrees in radians
        
        # Create array of angles for this scan
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        
        # Find angles within the front cone (around 0 degrees)
        # Handle angle wrapping: angles near 0 could be slightly negative or positive
        front_angles = np.where(np.abs(angles) <= front_cone_angle)[0]
        
        if len(front_angles) > 0:
            front_ranges = np.array(msg.ranges)[front_angles]
            # Filter out invalid readings (inf, nan, or beyond max range)
            valid_ranges = front_ranges[np.isfinite(front_ranges) & (front_ranges < msg.range_max)]
            
            if len(valid_ranges) > 0:
                min_distance = np.min(valid_ranges)
                
                if min_distance < self.emergency_stop_dist:
                    self.emergency_stop = True
                    self.get_logger().warn(f'EMERGENCY STOP: Obstacle at {min_distance:.2f}m')
                elif min_distance < self.slow_down_dist:
                    self.slow_down = True
                    self.get_logger().info(f'SLOW DOWN: Obstacle at {min_distance:.2f}m')
                elif min_distance < self.warning_dist:
                    self.warning = True
                    self.get_logger().info(f'WARNING: Obstacle at {min_distance:.2f}m')
    
    def depth_callback(self, msg):
        """Process depth camera data for obstacle detection"""
        self.depth_data = msg
        
        # Convert PointCloud2 to numpy array for processing
        # This is a simplified version - you may need more sophisticated processing
        try:
            points = self.pointcloud2_to_array(msg)
            if points is not None:
                # Check points in front of robot (simplified)
                front_points = points[points[:, 0] > 0]  # x > 0 means in front
                if len(front_points) > 0:
                    distances = np.sqrt(front_points[:, 0]**2 + front_points[:, 1]**2)
                    min_distance = np.min(distances)
                    
                    if min_distance < self.emergency_stop_dist:
                        self.emergency_stop = True
                        self.get_logger().warn(f'EMERGENCY STOP (Depth): Obstacle at {min_distance:.2f}m')
        except Exception as e:
            self.get_logger().debug(f'Depth processing error: {e}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            # Extract point data
            points = []
            for point in struct.iter_unpack('ffff', cloud_msg.data):
                points.append(point[:3])  # x, y, z
            return np.array(points)
        except:
            return None
    
    def cmd_vel_callback(self, msg):
        """Filter velocity commands based on safety status"""
        if self.emergency_stop:
            # Emergency stop - zero velocity
            filtered_msg = Twist()
            self.get_logger().warn('EMERGENCY STOP: Zeroing velocity command')
        elif self.slow_down:
            # Slow down - reduce velocity by 50%
            filtered_msg = Twist()
            filtered_msg.linear.x = msg.linear.x * 0.5
            filtered_msg.angular.z = msg.angular.z * 0.5
            self.get_logger().info('SLOW DOWN: Reducing velocity by 50%')
        else:
            # Normal operation
            filtered_msg = msg
        
        self.filtered_cmd_vel_pub.publish(filtered_msg)
    
    def safety_check(self):
        """Periodic safety status check and reset"""
        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = not self.emergency_stop  # True if safe
        self.safety_status_pub.publish(safety_msg)
        
        # Reset flags if no recent warnings
        if not self.emergency_stop and not self.slow_down and not self.warning:
            return
        
        # Reset flags periodically (simplified logic)
        self.emergency_stop = False
        self.slow_down = False
        self.warning = False

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 