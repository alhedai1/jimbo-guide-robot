import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point
import serial
import time
import numpy as np
from scipy.optimize import minimize

# configuring tags and anchor (nmt, nmi, nis) is done separately
# tags - 4 nmt
# anchor - 1 nmi
# network id - nis 1234 (both tags and anchor)

# tag positions:
tags = [
    (-0.30,  0.465),  # Front Left
    ( 0.30,  0.465),  # Front Right
    (-0.30, -0.465),  # Back Left
    ( 0.30, -0.465),  # Back Right
]

class UWBInterfaceNode(Node):
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
        time.sleep(2)

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
        self.pos_pub = self.create_publisher(Point, 'uwb_rel_position', 10)
        self.timer = self.create_timer(0.1, self.request_sensor_data)  # 10Hz

    def request_sensor_data(self):
        # Read from all serial ports with non-blocking approach
        for i, tag in enumerate(self.tags):
            if tag is None:
                continue  # Skip failed connections
                
            try:
                # response = tag.readline().decode('utf-8', errors='ignore').strip() # check output format
                # self.parse_response(response, i)
                # Check if data is available
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
        pos = self.estimate_tag_position(tags, self.distances)
        pos_msg = Point()
        pos_msg.x = pos[0]
        pos_msg.y = pos[1]
        pos_msg.z = 0.0  # Assuming 2D position
        self.pos_pub.publish(pos_msg)

        robot_center = np.mean(tags, axis=0)
        distance_to_person = np.linalg.norm(np.array(pos) - robot_center)

        if not hasattr(self, 'center_dist_pub'):
            self.center_dist_pub = self.create_publisher(Float32, 'uwb_person_distance', 10)
        dist_msg = Float32()
        dist_msg.data = float(distance_to_person)
        self.center_dist_pub.publish(dist_msg)

    def parse_response(self, line, tag_index):
        # for line in response.splitlines():
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
        # Check for valid distances
        # if any(d <= 0 or d > 10 for d in distances):  # 10m max reasonable distance
        #     return [0.0, 0.0]  # Return center if invalid
        
        def loss(p):
            x, y = p
            return sum((np.linalg.norm(np.array([x, y]) - np.array(tag)) - d) ** 2 for tag, d in zip(tags, distances))

        initial_guess = [0.0, 0.0]
        result = minimize(loss, initial_guess, method='L-BFGS-B', 
                         bounds=[(-5, 5), (-5, 5)])  # Reasonable bounds
        
        if result.success:
            return result.x
        # else:
        #     return [0.0, 0.0]  # Return center if optimization fails

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