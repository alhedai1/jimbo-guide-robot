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
        self.tags = [serial.Serial(port, baudrate=baudrate, timeout=1) for port in tag_ports]
        time.sleep(2)

        self.distances = [0.0] * len(self.tags)
        self.dist_pub = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)
        self.pos_pub = self.create_publisher(Point, 'uwb_rel_position', 10)
        self.timer = self.create_timer(0.1, self.request_sensor_data)  # 10Hz

        for tag in self.tags:
            tag.write(b'reset\r')
        time.sleep(1)

        for tag in self.tags:
            tag.write(b'\r')
            tag.write(b'\r')
        time.sleep(1)

        for tag in self.tags:
            tag.reset_input_buffer()
            tag.reset_output_buffer()
        time.sleep(1)
        
        for tag in self.tags:
            tag.write(b'lec\r')
        time.sleep(1)

        # self.tag.write(b'lec\r') # should query anchors on robot, not tag (tag on person, powered by battery)

    def request_sensor_data(self):
        for i, tag in enumerate(self.tags):
            try:
                response = tag.readline().decode('utf-8', errors='ignore').strip() # check output format
                self.parse_response(response, i)
            except Exception as e:
                self.get_logger().error(f"Serial error on tag {i}: {e}")
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
        def loss(p):
            x, y = p
            return sum((np.linalg.norm(np.array([x, y]) - np.array(tag)) - d) ** 2 for tag, d in zip(tags, distances))

        initial_guess = [0.0, 0.0]
        result = minimize(loss, initial_guess)
        return result.x

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