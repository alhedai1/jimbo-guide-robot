
# 2 components: front-end path planning & back-end optimization
# 
# 1: A* generate initial collision-free path using local grid map
# 2: euclidean distance field to facilitate b-spline optimization

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import minimize
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class BSOHFCController(Node):
    def __init__(self):
        super().__init__('bso_hfc_controller')

        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('dthr', 0.1)  # obstacle clearance threshold
        self.declare_parameter('v_max', 0.1)
        self.declare_parameter('a_max', 0.2)
        self.declare_parameter('target_distance_threshold', 0.1)
        self.declare_parameter('num_ctrl_points', 7)
        self.declare_parameter('num_spline_points', 30)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.dthr = self.get_parameter('dthr').value
        self.v_max = self.get_parameter('v_max').value
        self.a_max = self.get_parameter('a_max').value
        self.target_distance_threshold = self.get_parameter('target_distance_threshold').value
        self.num_ctrl_points = self.get_parameter('num_ctrl_points').value
        self.num_spline_points = self.get_parameter('num_spline_points').value

        self.track = False

        self.pose = None
        self.target = None
        self.laser = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_target = self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.subscription3 = self.create_subscription(Bool, '/tracking', self.tracking_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])

    def uwb_callback(self, msg):
        # self.target = np.array([msg.point.x, msg.point.y]) # uwb_filtered_position is in base_footprint frame
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

    def scan_callback(self, msg):
        self.laser = msg

    def control_loop(self):
        if self.pose is None or self.target is None or self.laser is None:
            return

        start = self.pose[:2]
        goal = self.target
        distance = np.linalg.norm(goal - start)
        if distance < self.target_distance_threshold:
            self.stop()
            return

        cps = np.linspace(start, goal, self.num_ctrl_points).T  # (2, N)
        fixed_start = cps[:, 0]
        fixed_end = cps[:, -1]
        initial_vars = cps[:, 2:-2].flatten()

        def penalty(x, y):
            return (x - y)**2 if x <= y else 0.0

        def compute_cost(vars):
            ctrl_pts = np.column_stack((
                fixed_start[:, None],
                cps[:, 1:2],
                vars.reshape(2, -1),
                cps[:, -2:-1],
                fixed_end[:, None]
            ))
            traj = self.eval_bspline(ctrl_pts, self.num_spline_points)

            cost = 0.0
            for pt in traj.T:
                cost += self.collision_cost(pt)

            # Smoothness (jerk)
            jerk = np.diff(ctrl_pts, n=3, axis=1)
            cost += np.sum(np.linalg.norm(jerk, axis=0)**2)

            # Dynamics
            dt = 0.1
            vels = np.diff(ctrl_pts, axis=1) / dt
            accs = np.diff(vels, axis=1) / dt
            for v in vels.T:
                cost += penalty(np.linalg.norm(v), self.v_max)
            for a in accs.T:
                cost += penalty(np.linalg.norm(a), self.a_max)

            return cost

        result = minimize(compute_cost, initial_vars, method='L-BFGS-B')
        if not result.success:
            self.get_logger().warn(f"Spline optimization failed: {result.message}")
            return

        opt_ctrl_pts = np.column_stack((
            fixed_start[:, None],
            cps[:, 1:2],
            result.x.reshape(2, -1),
            cps[:, -2:-1],
            fixed_end[:, None]
        ))

        traj = self.eval_bspline(opt_ctrl_pts, self.num_spline_points)
        self.follow_point(traj[:, 1])

    def eval_bspline(self, ctrl_pts, num_points):
        from scipy.interpolate import BSpline
        n = ctrl_pts.shape[1]
        degree = 3
        knots = np.concatenate(([0]*degree, np.linspace(0, 1, n - degree + 1), [1]*degree))
        t_vals = np.linspace(0, 1, num_points)
        spline_x = BSpline(knots, ctrl_pts[0], degree)(t_vals)
        spline_y = BSpline(knots, ctrl_pts[1], degree)(t_vals)
        return np.vstack((spline_x, spline_y))

    def collision_cost(self, pt):
        angle_min = self.laser.angle_min
        angle_inc = self.laser.angle_increment
        ranges = np.array(self.laser.ranges)

        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        r = np.hypot(dx, dy)
        a = np.arctan2(dy, dx) - self.pose[2]
        a = np.arctan2(np.sin(a), np.cos(a))
        idx = int((a - angle_min) / angle_inc)
        if 0 <= idx < len(ranges):
            dist = ranges[idx]
            if dist < self.dthr:
                return (self.dthr - dist)**2 * 100.0
        return 0.0

    def follow_point(self, pt):
        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        angle = np.arctan2(dy, dx)
        angle_diff = angle - self.pose[2]
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
