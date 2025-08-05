import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_geometry_msgs
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class UWBFollower(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')

        self.uwb_data = Point()
        self.uwb_target = None
        self.track = False

        self.subscription1 = self.create_subscription(Point, '/uwb_filtered_position', self.uwb_callback, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription3 = self.create_subscription(Bool, '/tracking', self.tracking_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dist_pub = self.create_publisher(Float32, '/dist', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # TF2 buffer and listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = rclpy.duration.Duration(seconds=5)

    def uwb_callback(self, msg):
        self.uwb_data = msg
        return
    
    def tracking_callback(self, msg):
        if msg.data == True:
            uwb_point = PoseStamped()
            uwb_point.header.stamp = self.get_clock().now().to_msg()
            uwb_point.header.frame_id = 'odom'
            uwb_point.pose.position = self.uwb_data
            uwb_point.pose.orientation.w = 1.0  # Identity quaternion
            self.uwb_target = uwb_point

    def control_loop(self):
        if self.uwb_target is None or self.laser_data is None:
            return
        
        # odom_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=self.tf_timeout)
        # odom_x = odom_tf.transform.translation.x # 0.0
        # odom_y = odom_tf.transform.translation.y # 0.0
        try:
            self.uwb_target.header.stamp = rclpy.time.Time().to_msg()
            target = self.tf_buffer.transform(self.uwb_target, 'base_footprint', timeout=self.tf_timeout)
        except Exception as e:
            self.get_logger().error(f'Unexpected TF2 error: {e}')
            return

        # tx = self.uwb_target.x - odom_x
        # ty = self.uwb_target.y - odom_y
        tx = target.pose.position.x
        ty = target.pose.position.y
        # ty = 0

        # Target direction
        # tx = self.uwb_target.x # 1.0
        # ty = self.uwb_target.y # 0.0
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

def main(args=None):
    rclpy.init(args=args)
    node = UWBFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
