import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PointStamped
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
import tf_transformations
import tf2_geometry_msgs
import math
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class ScanToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('scan_to_occupancy_grid')

        self.declare_parameter('grid_size', 10.0)   # meters
        self.declare_parameter('resolution', 0.05)  # meters/cell

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid_size = self.get_parameter('grid_size').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.grid_dim = int(self.grid_size / self.resolution)

        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'base_footprint'
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = self.grid_dim
        self.occupancy_grid.info.height = self.grid_dim
        self.occupancy_grid.info.origin.position.x = -self.grid_size / 2
        self.occupancy_grid.info.origin.position.y = -self.grid_size / 2
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0

        self.grid = np.zeros((self.grid_dim, self.grid_dim), dtype=np.int8)

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.pub = self.create_publisher(OccupancyGrid, '/my_occupancy_grid', 10)

    def scan_callback(self, msg: LaserScan):
        self.grid.fill(0)  # clear grid

        try:
            trans = self.tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min < r < msg.range_max:
                    x_local = r * math.cos(angle)
                    y_local = r * math.sin(angle)
                    # Apply transform manually
                    x = trans.transform.translation.x + x_local * math.cos(tf_transformations.euler_from_quaternion([
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ])[2]) - y_local * math.sin(tf_transformations.euler_from_quaternion([
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ])[2])
                    y = trans.transform.translation.y + x_local * math.sin(tf_transformations.euler_from_quaternion([
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ])[2]) + y_local * math.cos(tf_transformations.euler_from_quaternion([
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w
                    ])[2])

                    gx = int((x + self.grid_size/2) / self.resolution)
                    gy = int((y + self.grid_size/2) / self.resolution)
                    if 0 <= gx < self.grid_dim and 0 <= gy < self.grid_dim:
                        self.grid[gy, gx] = 100
                angle += msg.angle_increment
        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            return

        self.occupancy_grid.data = self.grid.flatten().tolist()
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
