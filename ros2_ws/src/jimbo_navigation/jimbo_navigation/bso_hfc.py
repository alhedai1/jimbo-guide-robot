
# 2 components: front-end path planning & back-end optimization
# 
# 1: A* generate initial collision-free path using local grid map
# 2: euclidean distance field to facilitate b-spline optimization

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import minimize
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_geometry_msgs
import time
import math
from jimbo_navigation.astar_planner import AStarPlanner
from scipy.ndimage import distance_transform_edt

class BSOHFCController(Node):
    def __init__(self):
        super().__init__('bso_hfc_controller')

        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('dthr', 0.4)  # obstacle clearance threshold
        self.declare_parameter('v_max', 0.03)
        self.declare_parameter('a_max', 0.1)
        self.declare_parameter('target_distance_threshold', 0.2)
        self.declare_parameter('num_ctrl_points', 5)
        self.declare_parameter('num_spline_points', 15)

        self.robot_radius = self.get_parameter('robot_radius').value
        self.dthr = self.get_parameter('dthr').value
        self.v_max = self.get_parameter('v_max').value
        self.a_max = self.get_parameter('a_max').value
        self.target_distance_threshold = self.get_parameter('target_distance_threshold').value
        self.num_ctrl_points = self.get_parameter('num_ctrl_points').value
        self.num_spline_points = self.get_parameter('num_spline_points').value

        self.target_locked = True
        self.path_ready = False

        self.pose = None
        self.target = None
        self.laser = None

        self.occ_grid = None
        self.grid_res = None
        self.grid_origin = None
        self.edt = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_target = self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(Bool, '/capture_target', self.capture_callback, 10)
        self.sub_occupancy = self.create_subscription(OccupancyGrid, 'my_occupancy_grid', self.occupancy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/bso_hfc_path', 10)

        self.spline = None
        self.traj_index = 1
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.optimize_timer = self.create_timer(0.5, self.optimize_path)

        self.astar = AStarPlanner(self.occ_grid, self.grid_res, self.grid_origin)
    
    def capture_callback(self, msg):
        if msg.data:
            self.target_locked = False  # clear lock to allow next UWB to register
            self.path_ready = False
        return

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])

    def uwb_callback(self, msg):
        if not self.target_locked:
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            # self.get_logger().info(f"Transforming from {pose.header.frame_id} to odom")
            pose.header.stamp = rclpy.time.Time().to_msg()
            pose.pose.position = msg.point
            pose.pose.orientation.w = 1.0
            odom_pose = self.tf_buffer.transform(pose, 'odom', timeout=rclpy.duration.Duration(seconds=1.0))
            self.target = np.array([odom_pose.pose.position.x, odom_pose.pose.position.y])
            self.target_locked = True

    def scan_callback(self, msg):
        self.laser = msg
    
    def occupancy_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.occ_grid = data
        self.grid_res = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.edt = self.compute_edt(self.occ_grid, self.grid_res)
        self.astar.update_grid(data, self.grid_res, self.grid_origin)

    def optimize_path(self):
        if self.pose is None or self.target is None or self.laser is None or self.occ_grid is None:
            return

        if self.path_ready:
            return
        
        self.path_ready = True

        start = self.pose[:2]
        goal = self.target
        distance = np.linalg.norm(goal - start)

        self.get_logger().info(f'start: {start}, goal: {goal}, distance: {distance}')
        # return

        if distance < self.target_distance_threshold:
            self.stop()
            return

        # if np.linalg.norm(goal - start) < 0.5:
        #     # Simple linear path fallback
        #     control = [start + (goal - start) * i / (self.num_points + 1) for i in range(1, self.num_points + 1)]
        #     path = [start] + control + [goal]
        #     self.get_logger().info("Skipping optimization for short distance. Using straight line.")
        #     return path

        # straight initial path
        # cps = np.linspace(start, goal, self.num_ctrl_points).T  # (2, N)

        # A* initial path
        path = self.astar.plan(start, goal)
        if not path:
            self.get_logger().warn(f"A* failed to find path")
            return
        path_np = np.array(path).T
        # self.get_logger().info(f"Length of path: {len(path)}, Shape of Path: {path_np.shape}\nPath: {path}")
        # self.publish_spline_path(path_np)
        # self.get_logger().info(f"Published A* path")
        
        cps = path_np[:, ::max(1, len(path_np[0]) // self.num_ctrl_points)]

        fixed_start = cps[:, 0]
        fixed_end = cps[:, -1]
        initial_vars = cps[:, 1:-1].flatten()
        initial_vars += np.random.normal(scale=0.01, size=initial_vars.shape)

        def penalty(x, y):
            return (x - y)**2 if x <= y else 0.0

        def compute_cost(vars):
            if np.any(np.isnan(vars)) or np.any(np.isinf(vars)):
                self.get_logger().warn("Invalid control point values: NaN or Inf detected!")
                return 1e6

            ctrl_pts = np.column_stack((
                fixed_start[:, None],
                # cps[:, 1:2],
                vars.reshape(2, -1),
                # cps[:, -2:-1],
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
                v_norm = np.linalg.norm(v)
                if np.isnan(v_norm):
                    return 1e6
                cost += penalty(v_norm, self.v_max)
            for a in accs.T:
                a_norm = np.linalg.norm(a)
                if np.isnan(v_norm):
                    return 1e6
                cost += penalty(a_norm, self.a_max)
            
            if np.isnan(cost) or np.isinf(cost):
                print("NaN/inf detected in cost. Debug info:")
                print("ctrl_pts:", ctrl_pts)
                print("traj shape:", traj.shape)
                print("jerk shape:", jerk.shape)
                return 1e6  # Large penalty

            return cost

        # result = minimize(compute_cost, initial_vars, method='L-BFGS-B')
        start_time = time.time()
        result = minimize(
            compute_cost,
            initial_vars,
            method='L-BFGS-B',
            options={
                'maxiter': 15,
                'disp': False,
                'gtol': 1e-3,  # allow slightly larger gradients
                "ftol": 1e-6,        # Early stopping
                'maxls': 40,   # increase line search steps
            }
        )
        elapsed = time.time() - start_time
        self.get_logger().info(f"Spline optimized in {elapsed:.3f} seconds.")
        if not result.success:
            self.get_logger().warn(f"Spline optimization failed: {result.message}")
            return
        else:
            self.get_logger().info(f"Spline optimization SUCCESS")
            
            # return

        opt_ctrl_pts = np.column_stack((
            fixed_start[:, None],
            cps[:, 1:2],
            result.x.reshape(2, -1),
            cps[:, -2:-1],
            fixed_end[:, None]
        ))

        traj = self.eval_bspline(opt_ctrl_pts, self.num_spline_points)
        self.publish_spline_path(traj)
        self.spline = traj  # Save the result
        self.traj_index = 1

    def control_loop(self):
        if self.spline is None or self.pose is None:
            return
        if self.traj_index >= self.spline.shape[1]:
            self.stop()
            self.spline = None
            return
        if np.linalg.norm(self.pose[:2] - self.target) < self.target_distance_threshold:
            self.stop()
            return

        # index = min(self.traj_index + 3, self.spline.shape[1] - 1)
        index = self.traj_index
        point_to_follow = self.spline[:, index]
        if np.linalg.norm(self.pose[:2] - point_to_follow) < 0.1:
            self.traj_index += 1
            index = self.traj_index
        point_to_follow = self.spline[:, index]
        self.follow_point(point_to_follow)

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
        # pt: (x, y) in world coordinates
        # angle_min = self.laser.angle_min
        # angle_inc = self.laser.angle_increment
        # ranges = np.array(self.laser.ranges)

        # dx = pt[0] - self.pose[0]
        # dy = pt[1] - self.pose[1]
        # r = np.hypot(dx, dy)
        # a = np.arctan2(dy, dx) - self.pose[2]
        # a = np.arctan2(np.sin(a), np.cos(a))
        # idx = int((a - angle_min) / angle_inc)
        # if 0 <= idx < len(ranges):
        #     dist = ranges[idx]
        #     margin = 0.2  # safe margin
        #     cost = np.exp(-5 * (dist - self.dthr + margin))  # smooth decay
        #     return cost if dist < self.dthr + margin else 0.0
        # return 0.0

        gx = int((pt[0] - self.grid_origin[0]) / self.grid_res)
        gy = int((pt[1] - self.grid_origin[1]) / self.grid_res)

        if 0 <= gx < self.edt.shape[1] and 0 <= gy < self.edt.shape[0]:
            dist = self.edt[gy, gx]
            if dist < self.dthr:
                return np.exp(-5 * (dist - self.dthr))  # Higher cost closer to obstacle
            else:
                return 0.0  # Safe
        else:
            return 1e3  # Outside map → heavy penalty

    def follow_point(self, pt):
        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - self.pose[2]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        distance = np.hypot(dx, dy)

        self.get_logger().info(f"Angle difference: {np.degrees(angle_diff)}")

        # Parameters
        max_lin_vel = 0.05  # m/s
        max_ang_vel = 0.1  # rad/s
        angle_gain = 1.0   # angular proportional gain
        slowdown_angle_thresh = 1.0  # rad
        min_lin_vel = 0.005

        # Adjust linear speed based on angle error
        speed_scale = max(0.0, 1.0 - abs(angle_diff) / slowdown_angle_thresh)
        cmd = Twist()
        cmd.linear.x = max(min_lin_vel, max_lin_vel * speed_scale) if distance > 0.05 else 0.0
        cmd.angular.z = np.clip(angle_gain * angle_diff, -max_ang_vel, max_ang_vel)

        self.get_logger().info("Computing cmd command")

        self.pub_cmd.publish(cmd)

    def compute_edt(self, grid, resolution):
        # 1 = obstacle, 0 = free → invert to compute distance to obstacles
        binary_obstacles = (grid == 1)
        free_space = np.logical_not(binary_obstacles)
        edt_cells = distance_transform_edt(free_space)  # ~binary_obstacles: where free
        edt_meters = edt_cells * resolution  # convert cells → meters
        return edt_meters

    def stop(self):
        self.pub_cmd.publish(Twist())
    
    def publish_spline_path(self, traj):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'  # Adjust if you're in another frame

        for i in range(traj.shape[1]):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = traj[0, i]
            pose.pose.position.y = traj[1, i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # no rotation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)




def main(args=None):
    rclpy.init(args=args)
    node = BSOHFCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
