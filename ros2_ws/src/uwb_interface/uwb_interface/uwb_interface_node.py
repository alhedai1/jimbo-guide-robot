import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
import serial
import time
import numpy as np
from scipy.optimize import minimize
from typing import Optional, Tuple
from tf2_ros import TransformBroadcaster
from collections import deque
from statistics import mean, stdev
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from uwb_interface.ukf import UWB_UKF
import csv

# configuring tags and anchor (nmt, nmi, nis) is done separately
# tags - 4 nmt
# anchor - 1 nmi
# network id - nis 1234 (both tags and anchor)

# tag positions: (+x is forward, +y is left)
tag_positions = [
    ( 0.2325,  0.29),  # Front Left
    ( 0.2325, -0.29),  # Front Right
    (-0.2325,  0.315),  # Back Left
    (-0.2325, -0.315),  # Back Right
]

class UWBInterfaceNode(Node):
    def __init__(self):
        super().__init__('uwb_interface_node')

        self.declare_parameter('tag_ports', ['/dev/uwb_front_left', '/dev/uwb_front_right', '/dev/uwb_back_left', '/dev/uwb_back_right']) # left front, right front, left back, right back
        self.declare_parameter('baudrate', 115200)

        tag_ports = self.get_parameter('tag_ports').get_parameter_value().string_array_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.tags = [] # list of Serial objects
        for i, port in enumerate(tag_ports):
            try:
                tag = serial.Serial(port, baudrate=baudrate, timeout=1)
                self.tags.append(tag)
                self.get_logger().info(f"Connected to UWB device {i} on {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {port}: {e}")
                self.tags.append(None)
        time.sleep(0.5)

        # Initialize UWB devices
        for i, tag in enumerate(self.tags):
            if tag is not None:
                try:
                    tag.write(b'reset\r')
                    time.sleep(0.5)
                    
                    tag.write(b'\r')
                    tag.write(b'\r')
                    time.sleep(0.5)

                    tag.reset_input_buffer()
                    tag.reset_output_buffer()
                    time.sleep(0.5)
                    
                    tag.write(b'lec\r')
                    time.sleep(1)

                    self.get_logger().info(f"Initialized UWB device {i}")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize UWB device {i}: {e}")

        self.distances = [0.0] * len(self.tags)
        self.dist_pub = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)
        self.pos_pub = self.create_publisher(PointStamped, 'uwb_filtered_position', 10)
        self.timer = self.create_timer(0.1, self.request_sensor_data)  # 10 hz

        # buffer for moving average filter
        self.position_buffer = deque(maxlen=10)

        # for debugging (getting avg & stddev)
        self.pos_history = deque(maxlen=10)  # 10 seconds of data at 10Hz
        self.dist_history = deque(maxlen=100)
        self.stats_timer = self.create_timer(1.0, self.compute_position_stats)
        self.dist_stats_timer = self.create_timer(1.0, self.compute_distance_stats)

        # for publishing transform from robot to person
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # initialize UKF filter
        self.ukf_tracker = UWB_UKF(tag_positions)
        self.ukf_last_time = self.get_clock().now()
        self.record = False

        self.initial = self.get_clock().now().nanoseconds

    def request_sensor_data(self):
        for i, tag in enumerate(self.tags):
            if tag is None:
                continue

            try:
                if tag.in_waiting > 0:
                    while tag.in_waiting > 0:
                        response = tag.readline().decode('utf-8', errors='ignore').strip()
                        if response:  
                            self.parse_response(response, i)
                else:
                    tag.timeout = 0.01
                    response = tag.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.parse_response(response, i)
                    tag.timeout = 1
            except Exception as e:
                self.get_logger().error(f"Serial error on tag {i}: {e}")
                # reset connection
                try:
                    tag.reset_input_buffer()
                    tag.reset_output_buffer()
                except:
                    pass
        
        # Publish 4 distances (for debugging)
        dist_msg = Float32MultiArray()
        dist_msg.data = self.distances
        # self.get_logger().info(f"Distances: {self.distances} | Time: {self.get_clock().now().nanoseconds-self.initial}")
        self.dist_pub.publish(dist_msg)
        self.dist_history.append(self.distances)

        # # !! skip until all distances are valid
        # for dist in self.distances:
        #     if dist <= 0.0:
        #         return

        # # Estimate position (multilateration, no filtering)
        # unfiltered_pos = self.estimate_tag_position(tag_positions, self.distances)
        # # self.get_logger().info(f"unfiltered_pos: {unfiltered_pos} | Time: {self.get_clock().now().nanoseconds-self.initial}")

        # use valid distances only
        valid_tags = []
        valid_distances = []
        for tag, dist in zip(tag_positions, self.distances):
            if dist > 0.0:
                valid_tags.append(tag)
                valid_distances.append(dist)
        if len(valid_distances) < 3:
            return
        
        # Estimate target position from 4 distances using UKF
        now = self.get_clock().now()
        dt = now - self.ukf_last_time
        self.ukf_last_time = now
        self.ukf_tracker.update(valid_tags, valid_distances, dt)
        ukf_filtered_pos = self.ukf_tracker.get_state()[:2]
        
        # moving average w/ 10 readings
        # self.position_buffer.append(ukf_filtered_pos)
        # ukf_filtered_pos = np.mean(self.position_buffer, axis=0)
        
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'uwb_base'
        # self.get_logger().info(f"ukf_Position: {ukf_filtered_pos[0]}, {ukf_filtered_pos[1]} | Time: {self.get_clock().now().nanoseconds-self.initial}")
        pos_msg.point.x = ukf_filtered_pos[0] + 0.3925
        pos_msg.point.y = ukf_filtered_pos[1]
        self.pos_history.append((pos_msg.point.x, pos_msg.point.y))
        # if self.get_clock().now().nanoseconds - self.initial > 8000000000:
        # self.get_logger().info(f"xy_Position: {pos_msg.point.x}, {pos_msg.point.y} | Time: {self.get_clock().now().nanoseconds-self.initial}")
        self.pos_pub.publish(pos_msg)

        # Publish TF of person position relative to robot
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'uwb_base'
        transform.child_frame_id = 'uwb_person'
        transform.transform.translation.x = pos_msg.point.x
        transform.transform.translation.y = pos_msg.point.y
        # if self.get_clock().now().nanoseconds - self.initial > 8000000000:
        self.tf_broadcaster.sendTransform(transform)

    def parse_response(self, line, tag_index):
        if 'DIST' in line:
            try:
                parts = line.split(',')
                # addr = parts[3]
                dist = float(parts[7])
                self.distances[tag_index] = dist
                # qf = int(parts[2].split(':')[1].strip())
            except Exception as e:
                self.get_logger().error(f"Serial error on tag {tag_index}: {e}")
    
    def estimate_tag_position(self, tags, distances):
        
        def loss(p):
            x, y = p
            return sum((np.linalg.norm(np.array([x, y]) - np.array(tag)) - d) ** 2 for tag, d in zip(tags, distances))

        initial_guess = [0.0, 0.0]
        result = minimize(loss, initial_guess, method='L-BFGS-B', 
                         bounds=[(-5, 5), (-5, 5)])  # Reasonable bounds
        
        if result.success:
            return result.x
        # else:
        #     return [0.0, 0.0]
    
    def compute_position_stats(self):
        if len(self.pos_history) < 10:
            return
        xs = [p[0] for p in self.pos_history]
        ys = [p[1] for p in self.pos_history]

        mean_x = mean(xs)
        mean_y = mean(ys)
        std_x = stdev(xs)
        std_y = stdev(ys)

        # self.get_logger().info(
        #     f"UWB STATS — Mean: ({mean_x:.2f}, {mean_y:.2f}) | StdDev: ({std_x*100:.1f}cm, {std_y*100:.1f}cm)"
        # )
    
    def compute_distance_stats(self):
        if len(self.dist_history) < 10:
            return
        dists = np.array(self.dist_history)  # shape (N, 4)
        std_devs = np.std(dists, axis=0)
        # self.get_logger().info(f"{std_devs}")
        variances = std_devs ** 2
        means = np.mean(dists, axis=0)
        
        # self.get_logger().info(f"means: {means[0]:.2f}, {means[1]:.2f}, {means[2]:.2f}, {means[3]:.2f}")
        # self.get_logger().info(f"stddev: {std_devs[0]*100}, {std_devs[1]*100}, {std_devs[2]*100}, {std_devs[3]*100}")

    def destroy_node(self):
        for tag in self.tags:
            if tag.is_open:
                tag.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UWBInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()