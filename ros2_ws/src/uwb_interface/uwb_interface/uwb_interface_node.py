import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Point
import serial
import time
import numpy as np
from scipy.optimize import minimize

# configuring tag and anchors (nmt, nmi, nis) is done separately
# tag - nmt
# anchors - 1 nmi, 3 nma
# network id - nis 1234 (both tag and anchors)

# anchor positions:
anchors = [(-1, 1), (1, 1), (-1, -1), (1, -1)]  # Example positions

class UWBInterfaceNode(Node):
    def __init__(self):
        super().__init__('uwb_interface_node')
        # Declare parameters for ports
        # self.declare_parameter('anchor_ports', ['/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyUSB4']) # left front, right front, left back, right back
        self.declare_parameter('anchor_ports', ['/dev/ttyUSB1']) # testing with one anchor
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        anchor_ports = self.get_parameter('anchor_ports').get_parameter_value().string_array_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open serial connections
        self.anchors = [serial.Serial(port, baudrate=baudrate, timeout=1) for port in anchor_ports]
        time.sleep(2)

        self.distances = [0.0] * len(self.anchors)
        self.dist_pub = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)
        self.pos_pub = self.create_publisher(Point, 'uwb_position', 10)
        self.timer = self.create_timer(0.1, self.request_sensor_data)  # 10Hz

        for anchor in self.anchors:
            anchor.write(b'reset\r')
        time.sleep(1)

        for anchor in self.anchors:
            anchor.write(b'\r')
        time.sleep(1)

        # self.tag.write(b'lec\r') # should query anchors on robot, not tag (tag on person, powered by battery)

    def request_sensor_data(self):
        for i, anchor in enumerate(self.anchors):
            try:
                anchor.reset_input_buffer()
                anchor.write(b'le\r')
                response = anchor.read_until(b'>').decode('utf-8', errors='ignore') # check output format
                self.parse_response(response, i)
            except Exception as e:
                self.get_logger().error(f"Serial error on anchor {i}: {e}")
        # Publish distances
        dist_msg = Float32MultiArray()
        dist_msg.data = self.distances
        self.dist_pub.publish(dist_msg)
        # Estimate position
        pos = self.estimate_tag_position(anchors, self.distances)
        pos_msg = Point()
        pos_msg.x = pos[0]
        pos_msg.y = pos[1]
        pos_msg.z = 0.0  # Assuming 2D position
        self.pos_pub.publish(pos_msg)

    def parse_response(self, response, anchor_index):
        for line in response.splitlines():
            if 'Range' in line:
                try:
                    parts = line.split(',')
                    addr = parts[0].split(':')[1].strip()
                    dist = float(parts[1].split(':')[1].strip().split()[0])
                    self.distances[anchor_index] = dist
                    qf = int(parts[2].split(':')[1].strip())
                except Exception as e:
                    self.get_logger().error(f"Serial error on anchor {anchor_index}: {e}")
    
    def estimate_tag_position(self, anchors, distances):
        def loss(p):
            x, y = p
            return sum((np.linalg.norm(np.array([x, y]) - np.array(anchor)) - d) ** 2 for anchor, d in zip(anchors, distances))

        initial_guess = [0.0, 0.0]
        result = minimize(loss, initial_guess)
        return result.x

    def destroy_node(self):
        for anchor in self.anchors:
            if anchor.is_open:
                anchor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UWBInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()