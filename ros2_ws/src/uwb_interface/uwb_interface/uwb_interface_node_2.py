import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PointStamped, TransformStamped
import serial
import time
import numpy as np
from scipy.optimize import minimize
from typing import Optional, Tuple
from tf2_ros import TransformBroadcaster
from collections import deque
from statistics import mean, stdev

# forward: +x, left: +y

# tag positions:
tags = [
    (0.465,   0.3),  # Front Left
    (0.465,  -0.3),  # Front Right
    (-0.465,  0.3),  # Back Left
    (-0.465, -0.3),  # Back Right
]

class UWBInterfaceNode2(Node):
    def __init__(self):
        super().__init__('uwb_interface_node')
        # Declare parameters for ports
        self.declare_parameter('tag_ports', ['/dev/uwb_front_left', '/dev/uwb_front_right', '/dev/uwb_back_left', '/dev/uwb_back_right']) # left front, right front, left back, right back
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        tag_ports = self.get_parameter('tag_ports').get_parameter_value().string_array_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial connections
        # self.tags = [serial.Serial(port, baudrate=baudrate, timeout=1) for port in tag_ports]
        self.tags = []
        for i, port in enumerate(tag_ports):
            try:
                tag = serial.Serial(port, baudrate=baudrate, timeout=1)
                self.tags.append(tag)
                self.get_logger().info(f"Connected to UWB device {i} on {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to {port}: {e}")
                # Create a dummy serial object to maintain indexing
                self.tags.append(None)
        time.sleep(0.5)

        # Initialize UWB devices
        for i, tag in enumerate(self.tags):
            if tag is not None:
                try:
                    # Send initialization commands
                    tag.write(b'reset\r')
                    time.sleep(0.5)
                    
                    tag.write(b'\r')
                    tag.write(b'\r')
                    time.sleep(0.5)

                    # Clear any existing data
                    tag.reset_input_buffer()
                    tag.reset_output_buffer()
                    time.sleep(0.5)
                    
                    tag.write(b'lec\r')
                    time.sleep(0.5)

                    self.get_logger().info(f"Initialized UWB device {i}")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize UWB device {i}: {e}")

        self.distances = [0.0] * len(self.tags)
        self.dist_pub = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)
        self.pos_pub = self.create_publisher(PointStamped, 'uwb_filtered_position', 10)
        self.timer = self.create_timer(0.1, self.request_sensor_data)  # 10Hz

        self.tf_broadcaster = TransformBroadcaster(self)

        # Kalman Filter init
        self.kalman_state = np.array([0.0, 0.0])  # initial position
        self.kalman_cov = np.eye(2) * 0.1         # initial covariance
        self.kalman_process_noise = np.eye(2) * 0.05
        self.kalman_measurement_noise = np.eye(2) * 0.2

        self.pos_history = deque(maxlen=100)  # 10 seconds of data at 10Hz
        self.stats_timer = self.create_timer(1.0, self.compute_position_stats)

    def request_sensor_data(self):
        # Read from all serial ports with non-blocking approach
        for i, tag in enumerate(self.tags):
            if tag is None:
                continue  # Skip failed connections
                
            try:
                if tag.in_waiting > 0:
                    # Read all available lines
                    while tag.in_waiting > 0:
                        response = tag.readline().decode('utf-8', errors='ignore').strip()
                        if response:  # Only process non-empty lines
                            self.parse_response(response, i)
                else:
                    # If no data available, try a quick read with short timeout
                    tag.timeout = 0.01  # 10ms timeout
                    response = tag.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.parse_response(response, i)
                    tag.timeout = 1  # Reset to original timeout
            except Exception as e:
                self.get_logger().error(f"Serial error on tag {i}: {e}")
                # Try to reset the connection
                try:
                    tag.reset_input_buffer()
                    tag.reset_output_buffer()
                except:
                    pass
        
        # Publish distances
        dist_msg = Float32MultiArray()
        dist_msg.data = self.distances
        self.dist_pub.publish(dist_msg)
    
        # Estimate position
        pos = self.compute_position(self.distances)
        if pos is not None:
            x, y = self.apply_kalman_filter(*pos)
            self.pos_history.append((x, y))

            # Publish as PointStamped
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'base_footprint'  # or whatever your robot's frame is
            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = 0.0
            self.pos_pub.publish(point)

    def parse_response(self, line, tag_index):
        # for line in response.splitlines():
        if 'DIST' in line:
            try:
                parts = line.split(',')
                # addr = parts[3]
                dist = float(parts[7])
                # if 0.2 < dist < 10.0:  # sanity check (adjust as needed)
                self.distances[tag_index] = dist
                # qf = int(parts[2].split(':')[1].strip())
            except Exception as e:
                self.get_logger().error(f"Serial error on tag {tag_index}: {e}")

    def compute_position(self, distances) -> Optional[Tuple[float, float]]:
        if any(d <= 0 for d in distances):
            return None  # Invalid readings
        
        def error_fn(p):
            return sum((np.linalg.norm(p - np.array(tag)) - d) ** 2
                       for tag, d in zip(tags, distances))

        # Start guess at center
        result = minimize(error_fn, x0=np.array([0.0, 0.0]), method='L-BFGS-B')
        if result.success:
            return tuple(result.x)
        else:
            self.get_logger().warn("Minimization failed")
            return None
    
    def apply_kalman_filter(self, x, y):
        z = np.array([x, y])
        # Prediction step
        pred_state = self.kalman_state
        pred_cov = self.kalman_cov + self.kalman_process_noise

        # Kalman Gain
        K = pred_cov @ np.linalg.inv(pred_cov + self.kalman_measurement_noise)

        # Update
        self.kalman_state = pred_state + K @ (z - pred_state)
        self.kalman_cov = (np.eye(2) - K) @ pred_cov

        return self.kalman_state
    
    def compute_position_stats(self):
        if len(self.pos_history) < 10:
            return
        xs = [p[0] for p in self.pos_history]
        ys = [p[1] for p in self.pos_history]

        mean_x = mean(xs)
        mean_y = mean(ys)
        std_x = stdev(xs)
        std_y = stdev(ys)

        self.get_logger().info(
            f"UWB STATS — Mean: ({mean_x:.2f}, {mean_y:.2f}) | StdDev: ({std_x*100:.1f}cm, {std_y*100:.1f}cm)"
        )

    def destroy_node(self):
        for tag in self.tags:
            if tag.is_open:
                tag.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UWBInterfaceNode2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()