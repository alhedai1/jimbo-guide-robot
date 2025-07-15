#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from std_msgs.msg import Bool
from nav2_msgs.msg import SpeedLimit
import numpy as np
import math
from typing import Optional, Tuple
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations
from tf2_ros import Buffer, TransformListener
import tf2_ros
from tf_transformations import quaternion_from_euler

class UserFollower(Node):
    def __init__(self):
        super().__init__('user_follower')
        
        # Parameters
        self.declare_parameter('target_distance', 0.5)      # 0.5m from user
        # self.declare_parameter('target_position', (-0.5, 0.0)) # Target position of person relative to robot (x, y), (0.5m behind robot)
        # self.declare_parameter('target_angle', math.pi)  # Target angle of person relative to robot (radians), 180.0 means directly behind
        self.declare_parameter('max_linear_velocity', 0.3)  # 0.3 m/s
        self.declare_parameter('max_angular_velocity', 0.5) # 0.5 rad/s
        
        self.target_distance = self.get_parameter('target_distance').value
        # self.target_position = self.get_parameter('target_position').value 
        # self.target_angle = self.get_parameter('target_angle').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        
        # State variables
        self.user_position: Optional[Tuple[float, float]] = None
        self.user_heading: Optional[float] = 0.0  # Not used currently, assumed to be 0
        self.safety_status = True
        
        # Publishers
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        # self.filtered_pos_pub = self.create_publisher(Point, "filtered_pos", 10) # debugging
        
        # Subscribers
        self.safety_sub = self.create_subscription(Bool, 'safety_status', self.safety_callback, 10)
        self.uwb_sub = self.create_subscription(Point, "uwb_filtered_position", self.uwb_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(1, self.control_loop)  # 1Hz
        # self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.alpha = 0.2 # cmd filter coefficient
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0

        # self.nav = BasicNavigator()
        # self.nav.waitUntilNav2Active()

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)

        # TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = rclpy.duration.Duration(seconds=0.5)
        
        self.get_logger().info('User Follower initialized')
    
    def safety_callback(self, msg):
        """Handle safety status updates"""
        self.safety_status = msg.data

    def uwb_callback(self, msg):
        # read filtered position of person relative to robot
        self.user_position = (msg.x, msg.y) # x - forward, y - left
        return
        
    def control_loop(self): # 1 Hz
        if not self.safety_status:
            # Safety violation - stop
            self.stop_robot()
            return
        
        if self.user_position is None:
            # No user detected - stop and wait
            self.stop_robot()
            self.get_logger().info('No user detected - waiting')
            return
        
        # # Calculate control commands
        # cmd_vel = self.calculate_following_command()
        # self.cmd_vel_pub.publish(cmd_vel)

        # Navigate to user
        self.navigate_to_user()
    
    def navigate_to_user(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', 'uwb_person', rclpy.time.Time(), timeout=self.tf_timeout)
            # # User position in robot frame
            # user_x_robot = self.user_position[0] + 0.5
            # user_y_robot = self.user_position[1]
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            distance = math.sqrt(x*x + y*y)
            if (distance < 0.5):
                self.get_logger().info(f"User too close: {distance:.2f}m, stopping")
                self.stop_robot()
                return
            
            x_goal = x + 0.5
            y_goal = y + 0.0

            odom_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=self.tf_timeout)

            robot_odom_x = odom_tf.transform.translation.x
            robot_odom_y = odom_tf.transform.translation.y

            goal_global_x = robot_odom_x + x_goal
            goal_global_y = robot_odom_y + y_goal

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'odom'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_global_x
            goal_pose.pose.position.y = goal_global_y

            q = quaternion_from_euler(0.0, 0.0, 0.0)  # No rotation
            goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            self.get_logger().info(f"BEFORE SENDING GOAL")
            # self.nav.goToPose(goal_pose)

            self.goal_pub.publish(goal_pose)
            speed_limit = SpeedLimit()
            speed_limit.percentage = True
            speed_limit.speed_limit = 20.0  # 20% speed limit
            self.speed_limit_pub.publish(speed_limit)
            
            self.get_logger().info(f"Sent goal to ({goal_global_x:.2f}, {goal_global_y:.2f})")

        except Exception as e:
            self.get_logger().warn(f"Failed to compute goal: {e}")
            return

    def calculate_following_command(self):
        """Calculate velocity commands to follow user"""
        cmd_vel = Twist()
        
        if self.user_position is None:
            return cmd_vel
        
        # Target position of person in robot frame: (-0.5, 0)
        x_goal = self.user_position[0] + 0.5
        y_goal = self.user_position[1] 
        
        # # Error in position
        # error_x = self.user_position[0] - target_x
        # error_y = self.user_position[1] - target_y

        # # Distance error
        # distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # Current distance to user
        current_distance = math.sqrt(self.user_position[0]**2 + self.user_position[1]**2)

        if (current_distance < self.target_distance):
            return cmd_vel # Stop moving if too close
        
        # Optional: if too close, move away to maintain fixed distance
        
        # Distance error
        distance_error = current_distance - self.target_distance
        
        # Angular error (angle to user)
        angle_to_user = math.atan2(-self.user_position[1], self.user_position[0])

        if (abs(distance_error) < 0.05 and abs(angle_to_user) < 0.05):
            # stop moving
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            return cmd_vel
        
        # Simple proportional control
        linear_vel = 0.25 * distance_error * math.cos(angle_to_user)  # P controller for distance
        angular_vel = 0.5 * angle_to_user  # P controller for angle
        
        # Limit velocities
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        
        cmd_vel.linear.x = self.alpha * linear_vel + (1 - self.alpha) * self.prev_linear_vel
        cmd_vel.angular.z = self.alpha * angular_vel + (1 - self.alpha) * self.prev_angular_vel

        self.prev_linear_vel = cmd_vel.linear.x
        self.prev_angular_vel = cmd_vel.angular.z

        # Deadzone
        if abs(cmd_vel.linear.x) < 0.05:
            cmd_vel.linear.x = 0.0
        if abs(cmd_vel.angular.z) < 0.05:
            cmd_vel.angular.z = 0.0

        # add acceleration ramping if necessary

        # self.get_logger().debug(f'Following user at {current_distance:.2f}m, target: {self.target_distance}m')
        
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