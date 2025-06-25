#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math
from typing import Optional, Tuple

class UserFollower(Node):
    def __init__(self):
        super().__init__('user_follower')
        
        # Parameters
        self.declare_parameter('target_distance', 1.0)      # 1m from user
        self.declare_parameter('max_linear_velocity', 0.5)  # 0.5 m/s
        self.declare_parameter('max_angular_velocity', 1.0) # 1.0 rad/s
        self.declare_parameter('person_detection_threshold', 0.3)  # 30cm radius
        
        self.target_distance = self.get_parameter('target_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.person_threshold = self.get_parameter('person_detection_threshold').value
        
        # State variables
        self.user_position: Optional[Tuple[float, float]] = None
        self.user_velocity: Optional[Tuple[float, float]] = None
        self.last_user_position: Optional[Tuple[float, float]] = None
        self.safety_status = True
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.safety_sub = self.create_subscription(Bool, 'safety_status', self.safety_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('User Follower initialized')
    
    def lidar_callback(self, msg):
        """Process lidar data to detect and track user"""
        # Simple person detection based on leg detection
        # This is a simplified approach - you may want to use more sophisticated methods
        
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.where((ranges > 0.1) & (ranges < 5.0))[0]
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) == 0:
            return
        
        # Convert to cartesian coordinates
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        
        # Simple clustering to find potential people
        people = self.detect_people(x_coords, y_coords)
        
        if people:
            # Find the closest person (assumed to be the user)
            closest_person = min(people, key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
            self.user_position = closest_person
            
            # Calculate user velocity (simplified)
            if self.last_user_position is not None:
                dt = 0.1  # 10Hz update rate
                dx = self.user_position[0] - self.last_user_position[0]
                dy = self.user_position[1] - self.last_user_position[1]
                self.user_velocity = (dx/dt, dy/dt)
            
            self.last_user_position = self.user_position
        else:
            self.user_position = None
            self.user_velocity = None
    
    def detect_people(self, x_coords, y_coords):
        """Simple person detection using clustering"""
        people = []
        
        # Group points that are close together
        points = list(zip(x_coords, y_coords))
        clusters = []
        
        for point in points:
            added_to_cluster = False
            for cluster in clusters:
                # Check if point is close to any point in cluster
                for cluster_point in cluster:
                    distance = math.sqrt((point[0] - cluster_point[0])**2 + 
                                       (point[1] - cluster_point[1])**2)
                    if distance < self.person_threshold:
                        cluster.append(point)
                        added_to_cluster = True
                        break
                if added_to_cluster:
                    break
            
            if not added_to_cluster:
                clusters.append([point])
        
        # Filter clusters that could be people (size and position)
        for cluster in clusters:
            if len(cluster) >= 3:  # Minimum points for a person
                # Calculate cluster center
                center_x = sum(p[0] for p in cluster) / len(cluster)
                center_y = sum(p[1] for p in cluster) / len(cluster)
                
                # Check if cluster is in front of robot and reasonable distance
                if center_x > 0.3 and center_x < 3.0:  # Between 30cm and 3m
                    people.append((center_x, center_y))
        
        return people
    
    def safety_callback(self, msg):
        """Handle safety status updates"""
        self.safety_status = msg.data
    
    def control_loop(self):
        """Main control loop for following user"""
        if not self.safety_status:
            # Safety violation - stop
            self.stop_robot()
            return
        
        if self.user_position is None:
            # No user detected - stop and wait
            self.stop_robot()
            self.get_logger().info('No user detected - waiting')
            return
        
        # Calculate control commands
        cmd_vel = self.calculate_following_command()
        self.cmd_vel_pub.publish(cmd_vel)
    
    def calculate_following_command(self):
        """Calculate velocity commands to follow user"""
        cmd_vel = Twist()
        
        if self.user_position is None:
            return cmd_vel
        
        # Current distance to user
        current_distance = math.sqrt(self.user_position[0]**2 + self.user_position[1]**2)
        
        # Distance error
        distance_error = current_distance - self.target_distance
        
        # Angular error (angle to user)
        angle_to_user = math.atan2(self.user_position[1], self.user_position[0])
        
        # Simple proportional control
        linear_vel = 0.5 * distance_error  # P controller for distance
        angular_vel = 1.0 * angle_to_user  # P controller for angle
        
        # Limit velocities
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Apply velocity limits based on safety
        if abs(distance_error) < 0.1:  # Close to target distance
            linear_vel *= 0.5  # Slow down when close
        
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        
        self.get_logger().debug(f'Following user at {current_distance:.2f}m, target: {self.target_distance}m')
        
        return cmd_vel
    
    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = UserFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 