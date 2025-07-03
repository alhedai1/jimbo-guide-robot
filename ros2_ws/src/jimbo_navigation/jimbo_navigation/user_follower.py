#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import math
from typing import Optional, Tuple

class UserFollower(Node):
    def __init__(self):
        super().__init__('user_follower')
        
        # Parameters
        self.declare_parameter('target_distance', 0.5)      # 0.5m from user
        self.declare_parameter('max_linear_velocity', 0.3)  # 0.3 m/s
        self.declare_parameter('max_angular_velocity', 0.5) # 0.5 rad/s
        
        self.target_distance = self.get_parameter('target_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        
        # State variables
        self.user_position: Optional[Tuple[float, float]] = None
        self.safety_status = True
        
        # Kalman filter state for user position [x, y, vx, vy]
        self.kalman_initialized = False
        self.kalman_state = np.zeros(4)  # [x, y, vx, vy]
        self.kalman_P = np.eye(4)
        self.kalman_Q = np.eye(4) * 0.01  # Process noise
        self.kalman_R = np.eye(2) * 0.1   # Measurement noise
        self.kalman_F = np.eye(4)
        self.kalman_H = np.zeros((2, 4))
        self.kalman_H[0, 0] = 1
        self.kalman_H[1, 1] = 1
        self.kalman_last_time = self.get_clock().now()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.filtered_pos_pub = self.create_publisher(Point, "filtered_pos", 10)
        
        # Subscribers
        self.safety_sub = self.create_subscription(Bool, 'safety_status', self.safety_callback, 10)
        self.uwb_sub = self.create_subscription(Point, "uwb_rel_position", self.uwb_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('User Follower initialized')
    
    def safety_callback(self, msg):
        """Handle safety status updates"""
        self.safety_status = msg.data

    def uwb_callback(self, msg):
        z = np.array([msg.x, msg.y])
        now = self.get_clock().now()
        dt = (now - self.kalman_last_time).nanoseconds * 1e-9
        if dt <= 0 or dt > 1.0:
            dt = 0.1  # fallback for first call or large jumps
        self.kalman_last_time = now

        if not self.kalman_initialized:
            self.kalman_state[:2] = z
            self.kalman_initialized = True
            self.user_position = (z[0], z[1])
            return

        # Update F for dt
        self.kalman_F[0, 2] = dt
        self.kalman_F[1, 3] = dt

        # Predict
        self.kalman_state = self.kalman_F @ self.kalman_state
        self.kalman_P = self.kalman_F @ self.kalman_P @ self.kalman_F.T + self.kalman_Q

        # Update
        y = z - self.kalman_H @ self.kalman_state
        S = self.kalman_H @ self.kalman_P @ self.kalman_H.T + self.kalman_R
        K = self.kalman_P @ self.kalman_H.T @ np.linalg.inv(S)
        self.kalman_state = self.kalman_state + K @ y
        self.kalman_P = (np.eye(4) - K @ self.kalman_H) @ self.kalman_P

        self.user_position = (self.kalman_state[0], self.kalman_state[1])
        self.filtered_pos_pub.publish(Point(x=self.user_position[0], y=self.user_position[1], z=0.0))
        
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
        if abs(distance_error) < 0.2:  # Close to target distance
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