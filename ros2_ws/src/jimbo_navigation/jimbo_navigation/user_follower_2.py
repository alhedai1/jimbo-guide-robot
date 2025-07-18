import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist, PointStamped, Point
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class UWBFollower(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')

        self.declare_parameter('robot_radius', 0.2)  # meters
        self.declare_parameter('max_lin_vel', 0.01)
        self.declare_parameter('max_ang_vel', 0.03)
        self.declare_parameter('sim_time', 1.0)  # seconds
        self.declare_parameter('dt', 0.1)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.sim_time = self.get_parameter('sim_time').value
        self.dt = self.get_parameter('dt').value

        self.uwb_data = Point()
        # self.uwb_target = Point(x=1.0, y=0.0, z=0.0)
        self.uwb_target = Point()
        self.laser_data = None
        self.track = False

        self.subscription1 = self.create_subscription(PointStamped, '/uwb_target', self.uwb_callback, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription2 = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.subscription3 = self.create_subscription(Bool, '/tracking', self.tracking_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dist_pub = self.create_publisher(Float32, '/dist', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = rclpy.duration.Duration(seconds=5)

    def uwb_callback(self, msg):
        self.uwb_data = msg.point
        return

    def lidar_callback(self, msg):
        self.laser_data = msg
    
    def tracking_callback(self, msg):
        if msg.data == True:
            self.uwb_target = self.uwb_data

    def control_loop(self):
        if self.uwb_target is None or self.laser_data is None:
            return
        
        odom_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=self.tf_timeout)
        odom_x = odom_tf.transform.translation.x # 0.0
        odom_y = odom_tf.transform.translation.y # 0.0

        # tx = self.uwb_target.x - odom_x
        # ty = self.uwb_target.y - odom_y
        # ty = 0

        # Target direction
        tx = self.uwb_target.x # 1.0
        ty = self.uwb_target.y # 0.0
        target_distance = math.hypot(tx, ty)
        # self.dist_pub.publish(Float32(data=target_distance))
        self.get_logger().info(f'distance: {target_distance}')
        target_angle = math.atan2(ty, tx)

        if target_distance < 0.1:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info('Reached Target!')
            return

        # Sample velocity commands
        best_score = float('-inf')
        best_cmd = Twist()

        for v in np.linspace(0.0, self.max_lin_vel, 5):
            for w in np.linspace(-self.max_ang_vel, self.max_ang_vel, 7):
                if self.is_trajectory_safe(v, w):
                    heading_score = -abs(target_angle - w * self.sim_time)  # prefer turning toward target
                    velocity_score = v
                    score = heading_score + velocity_score
                    if score > best_score:
                        best_score = score
                        best_cmd.linear.x = v
                        best_cmd.angular.z = w

        self.cmd_vel_pub.publish(best_cmd)

    def is_trajectory_safe(self, v, w):
        if v == 0.0 and abs(w) < 1e-3:
            return False

        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        ranges = np.array(self.laser_data.ranges)

        x, y, theta = 0.0, 0.0, 0.0
        for _ in np.arange(0, self.sim_time, self.dt):
            # simulate forward
            x += v * self.dt * math.cos(theta)
            y += v * self.dt * math.sin(theta)
            theta += w * self.dt

            # convert to polar
            r = math.hypot(x, y)
            a = math.atan2(y, x)
            idx = int((a - angle_min) / angle_increment)

            if 0 <= idx < len(ranges):
                if ranges[idx] < r + self.robot_radius:
                    return False  # too close to obstacle

        return True

def main(args=None):
    rclpy.init(args=args)
    node = UWBFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
