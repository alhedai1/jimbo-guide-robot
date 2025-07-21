import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import minimize
import time

class BSOHFCController(Node):
    def __init__(self):
        super().__init__('bso_hfc_controller')

        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('target_distance_threshold', 0.0)
        self.declare_parameter('spline_horizon', 1.5)  # meters
        self.declare_parameter('num_ctrl_points', 5)
        self.declare_parameter('num_spline_points', 30)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.target_distance_threshold = self.get_parameter('target_distance_threshold').value
        self.num_ctrl_points = self.get_parameter('num_ctrl_points').value
        self.num_spline_points = self.get_parameter('num_spline_points').value

        self.target = None
        self.pose = None
        self.laser = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_uwb = self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.subscription3 = self.create_subscription(Bool, '/tracking', self.tracking_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.track = False

        self.timer = self.create_timer(0.1, self.control_loop)

        # self.pub_cmd.publish(Twist())

    def uwb_callback(self, msg):
        if self.track:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            # self.get_logger().info(f"Transforming from {pose.header.frame_id} to odom")
            pose.header.stamp = rclpy.time.Time().to_msg()
            pose.pose.position = msg.point
            pose.pose.orientation.w = 1.0
            odom_pose = self.tf_buffer.transform(pose, 'odom', timeout=rclpy.duration.Duration(seconds=1.0))
            self.target = np.array([odom_pose.pose.position.x, odom_pose.pose.position.y])
            self.track = False

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])

    def scan_callback(self, msg):
        self.laser = msg
    
    def tracking_callback(self, msg):
        if msg.data == True:
            self.track = True

    def control_loop(self):
        if self.pose is None or self.target is None or self.laser is None:
            return

        start = self.pose[:2]
        goal = self.target
        distance_to_target = np.linalg.norm(goal - start)

        if distance_to_target < self.target_distance_threshold:
            self.stop()
            return

        # Initial guess: straight line control points
        cps_x = np.linspace(start[0], goal[0], self.num_ctrl_points)
        cps_y = np.linspace(start[1], goal[1], self.num_ctrl_points)
        cps_init = np.hstack((cps_x, cps_y))

        def cost_fn(ctrl_pts):
            ctrl_pts = ctrl_pts.reshape(2, -1)
            x_vals = np.linspace(0, 1, self.num_spline_points)
            traj = self.b_spline(ctrl_pts, x_vals)

            cost = 0.0
            for pt in traj.T:
                cost += self.obstacle_cost(pt)
            cost += self.smoothness_cost(ctrl_pts)
            cost += 10.0 * np.linalg.norm(traj[:, -1] - goal)  # end goal cost
            self.get_logger().info(f"Cost: {cost:.2f}, Goal dist: {np.linalg.norm(traj[:, -1] - goal):.2f}")
            return cost

        result = minimize(cost_fn, cps_init, method='L-BFGS-B')
        if not result.success:
            self.get_logger().warn('Spline optimization failed')
            return

        ctrl_pts_opt = result.x.reshape(2, -1)
        x_vals = np.linspace(0, 1, self.num_spline_points)
        traj = self.b_spline(ctrl_pts_opt, x_vals)

        next_pt = traj[:, 1]
        self.follow_point(next_pt)

    def b_spline(self, ctrl_pts, x_vals):
        from scipy.interpolate import BSpline
        n = ctrl_pts.shape[1]
        degree = 3
        kv = np.concatenate(([0]*degree, np.linspace(0, 1, n - degree + 1), [1]*degree))
        spline_x = BSpline(kv, ctrl_pts[0], degree)(x_vals)
        spline_y = BSpline(kv, ctrl_pts[1], degree)(x_vals)
        return np.vstack((spline_x, spline_y))

    def obstacle_cost(self, pt):
        return 0.0
        angle_min = self.laser.angle_min
        angle_inc = self.laser.angle_increment
        ranges = np.array(self.laser.ranges)

        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        r = np.hypot(dx, dy)
        a = np.arctan2(dy, dx) - self.pose[2]
        a = np.arctan2(np.sin(a), np.cos(a))  # normalize angle

        idx = int((a - angle_min) / angle_inc)
        if 0 <= idx < len(ranges):
            dist = ranges[idx]
            if dist < r + self.robot_radius:
                return 1000.0  # heavy penalty
        return 0.0

    def smoothness_cost(self, ctrl_pts):
        diffs = np.diff(ctrl_pts, axis=1)
        return np.sum(np.linalg.norm(diffs, axis=0)**2)

    def follow_point(self, pt):
        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - self.pose[2]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        cmd = Twist()
        cmd.linear.x = 0.2 if abs(angle_diff) < 0.5 else 0.0
        cmd.angular.z = 1.5 * angle_diff
        self.pub_cmd.publish(cmd)

    def stop(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = BSOHFCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()