
# 2 components: front-end path planning & back-end optimization
# 
# 1: A* generate initial collision-free path using local grid map
# 2: euclidean distance field to facilitate b-spline optimization

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Header
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import minimize
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_geometry_msgs
import time
import math
from jimbo_navigation.astar_planner import AStarPlanner
from jimbo_navigation.hybrid_astar import HybridAStarPlanner
from scipy.ndimage import distance_transform_edt
from scipy.ndimage import binary_dilation
from typing import Tuple, List
from scipy.ndimage import gaussian_filter1d
from jimbo_navigation.dwa import DWAController

class BSOHFCController(Node):
    def __init__(self):
        super().__init__('bso_hfc_controller')

        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('dthr', 0.2)  # obstacle clearance threshold
        self.declare_parameter('v_max', 0.03)
        self.declare_parameter('a_max', 0.1)
        self.declare_parameter('target_distance_threshold', 0.0) # adjust threshold (0.0 just for testing)
        self.declare_parameter('num_ctrl_points', 5)
        self.declare_parameter('num_spline_points', 15) # try 50 spline points

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
        # self.pose = np.array([0.0, 0.0, 0.0])
        # self.target = np.array([3.0, -1.0])
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
        self.sub_occupancy = self.create_subscription(OccupancyGrid, 'inflated_occupancy_grid', self.occupancy_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/bso_hfc_path', 10)
        self.astar_path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.edt_pub = self.create_publisher(OccupancyGrid, '/edt_grid', 10)

        self.spline = None
        self.traj_index = 1
        # self.control_timer = self.create_timer(0.05, self.control_loop)
        self.optimize_timer = self.create_timer(1, self.optimize_path)

        self.astar = HybridAStarPlanner(self.occ_grid, self.grid_res, self.grid_origin, self.get_logger())

        self.obstacles = None
        # self.dwa = DWAController()
        
    
    def capture_callback(self, msg):
        if msg.data:
            self.target_locked = False  # clear lock to allow next UWB to register
            self.path_ready = False
        return

    def goal_callback(self, msg: PoseStamped):
        self.target = np.array([msg.pose.position.x, msg.pose.position.y])

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])

    def uwb_callback(self, msg):
        # if not self.target_locked:
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        # self.get_logger().info(f"Transforming from {pose.header.frame_id} to odom")
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        odom_pose = self.tf_buffer.transform(pose, 'odom', timeout=rclpy.duration.Duration(seconds=1.0))
        self.target = np.array([odom_pose.pose.position.x, odom_pose.pose.position.y])
        # self.target_locked = True

    def scan_callback(self, msg):
        self.laser = msg
    
    def occupancy_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        self.occ_grid = np.array(data, dtype=np.int8).reshape((height, width))
        self.grid_res = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.edt = self.compute_edt(self.occ_grid, self.grid_res)

        obstacles = []
        for idx, value in enumerate(data):
            if value > 50:
                map_x = idx % width
                map_y = idx // width

                x = self.grid_origin[0] + (map_x + 0.5) * self.grid_res
                y = self.grid_origin[1] + (map_y + 0.5) * self.grid_res

                obstacles.append((x, y))
        if not self.obstacles:
            self.obstacles = obstacles
            self.dwa = DWAController(self.obstacles)

        self.publish_edt_as_grid(self.edt, self.grid_res, self.grid_origin)
        # inflated_grid = self.inflate_obstacles(self.occ_grid, int(0.2 / self.grid_res))
        self.astar.update_grid(self.occ_grid, self.grid_res, self.grid_origin)

    def optimize_path(self):
        if self.pose is None or self.target is None or self.laser is None or self.occ_grid is None:
            return

        if self.path_ready:
            return
        self.path_ready = True

        start = self.pose[:2]
        goal = self.target
        distance = np.linalg.norm(goal - start)

        self.get_logger().info(f'start: {start}, goal: {goal}, distance: {distance:.3f}')
        # return

        self.dwa.dwa_control(start, goal)

        if distance < self.target_distance_threshold:
            self.get_logger().info(f"Distance < target_distance_threshold. Stopping...")
            self.stop()
            return

        # A* initial path
        # self.get_logger().info(f"Starting to plan path...")
        start_time = time.time()
        path = self.astar.plan(start, goal)
        if not path:
            self.get_logger().warn(f"A* failed to find path")
            return
        elapsed = time.time() - start_time
        self.get_logger().info(f"Time to plan Hybrid-A* path: {elapsed:.2f} seconds")
        path_np = np.array(path).T
        # self.get_logger().info(f"Path_np: {path_np}")
        # self.num_ctrl_points = max(5, int(len(path_np[0]) // 2))
        # self.get_logger().info(f"Length of path: {len(path)}, Shape of Path: {path_np.shape}\nPath: {path}")
        self.publish_spline_path(path_np, True)
        # self.get_logger().info(f"Published A* path")
        return
        
        # self.get_logger().info(f"length of path_np: {path_np.shape}")
        cps = path_np[:, ::max(1, len(path_np[0]) // self.num_ctrl_points)] # doesn't guarantee that last point (goal) is included in control points
        # self.get_logger().info(f"length of cps: {cps.shape}")

        fixed_start = cps[:, 0]
        fixed_end = cps[:, -1]
        initial_vars = cps[:, 1:-1].flatten()
        # initial_vars += np.random.normal(scale=0.1, size=initial_vars.shape)
        # initial_vars = gaussian_filter1d(path_np, sigma=1.0, axis=1)[:, 1:-1].flatten()

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

            w_collision = 4.0
            w_jerk = 5.0
            w_dynamic = 1.0

            cost = 0.0
            for pt in traj.T:
                cost += w_collision * self.collision_cost(pt)
            
            # self.get_logger().info(f"Collision cost: {cost:.2f}")

            # Smoothness (jerk)
            jerk = np.diff(ctrl_pts, n=3, axis=1)
            cost += w_jerk * np.sum(np.linalg.norm(jerk, axis=0)**2)

            # Dynamics
            dt = 0.1
            vels = np.diff(ctrl_pts, axis=1) / dt
            accs = np.diff(vels, axis=1) / dt
            for v in vels.T:
                v_norm = np.linalg.norm(v)
                if np.isnan(v_norm):
                    return 1e6
                cost += w_dynamic * penalty(v_norm, self.v_max)
            for a in accs.T:
                a_norm = np.linalg.norm(a)
                if np.isnan(v_norm):
                    return 1e6
                cost += w_dynamic * penalty(a_norm, self.a_max)
            
            if np.isnan(cost) or np.isinf(cost):
                print("NaN/inf detected in cost. Debug info:")
                print("ctrl_pts:", ctrl_pts)
                print("traj shape:", traj.shape)
                print("jerk shape:", jerk.shape)
                return 1e6  # Large penalty

            # self.get_logger().info(f"Total cost: {cost:.2f}")
            return cost

        # result = minimize(compute_cost, initial_vars, method='L-BFGS-B')
        start_time = time.time()
        result = minimize(
            compute_cost,
            initial_vars,
            # method='L-BFGS-B',
            method='SLSQP',
            options={
                'maxiter': 30,
                'disp': False,
                'gtol': 1e-3,  # allow slightly larger gradients
                "ftol": 1e-6,  # Early stopping
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
            # cps[:, 1:2],
            result.x.reshape(2, -1),
            # cps[:, -2:-1],
            fixed_end[:, None]
        ))

        traj = self.eval_bspline(opt_ctrl_pts, self.num_spline_points)
        self.publish_spline_path(traj, False)
        self.spline = traj  # Save the result
        self.traj_index = 0
        self.get_logger().info(f"Shape of path: {traj.shape}")

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

        # use laser scan directly
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
        #     margin = 0.5  # safe margin
        #     cost = np.exp(5 * (dist - self.dthr + margin))  # smooth decay
        #     return cost if dist < self.dthr + margin else 0.0
        # return 0.0

        # use EDT
        # convert world to grid coordinates
        gx = int((pt[0] - self.grid_origin[0]) / self.grid_res)
        gy = int((pt[1] - self.grid_origin[1]) / self.grid_res)

        if 0 <= gx < self.edt.shape[1] and 0 <= gy < self.edt.shape[0]:
            dist = self.edt[gy, gx]
            # dist = max(1e-2, dist)
            if dist < (self.dthr + self.robot_radius):
                # return np.exp(-5 * (dist - self.dthr))  # Higher cost closer to obstacle
                return np.exp(10 * ((self.dthr + self.robot_radius) - dist)) 
                # return (self.dthr - dist)**2 # Try quadratic cost
            else:
                # return 0.0  # Safe
                return np.exp(-10 * (dist - (self.dthr + self.robot_radius)))
        else:
            return 1e3  # Outside map → heavy penalty

    def compute_edt(self, grid, resolution):
        """
        Converts an occupancy grid (with values 0–100 and -1) into an EDT (in meters).
        grid: 2D numpy array
        resolution: meters per cell
        """
        # Handle unknowns (-1) as obstacles or customize depending on use
        grid = grid.copy()
        grid[grid == -1] = 100  # Treat unknowns as obstacles

        # Define binary obstacle map: 1 = obstacle, 0 = free
        binary_obstacles = grid >= 50  # or >=100 for strict, >=50 more lenient

        # Invert: free space = True, obstacles = False
        free_space = np.logical_not(binary_obstacles)

        # Compute EDT
        edt_cells = distance_transform_edt(free_space)

        # Convert cell distances to meters
        edt_meters = edt_cells * resolution
        return edt_meters

    def inflate_obstacles(self, grid, radius_cells):
        return binary_dilation(grid, iterations=radius_cells).astype(np.uint8)
    
    def publish_spline_path(self, traj, astar):
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
        if astar:
            self.astar_path_pub.publish(path_msg)
        else:
            self.path_pub.publish(path_msg)


    def control_loop(self):
        if self.spline is None or self.pose is None:
            self.get_logger().info(f"Spline or Pose is None.")
            return
        if self.traj_index >= self.spline.shape[1]:
            self.stop()
            self.spline = None
            self.get_logger().info(f"Trajectory index > Spline length.")
            return
        if np.linalg.norm(self.pose[:2] - self.target) < self.target_distance_threshold:
            self.stop()
            return

        # index = min(self.traj_index + 3, self.spline.shape[1] - 1)
        index = self.traj_index
        point_to_follow = self.spline[:, index]
        while (np.linalg.norm(self.pose[:2] - point_to_follow) < 0.1):
            self.traj_index += 1
            if self.traj_index >= self.spline.shape[1]:
                break
            index = self.traj_index
            point_to_follow = self.spline[:, index]
        # point_to_follow = self.spline[:, index]
        # self.get_logger().info(f"Trajectory index: {self.traj_index}")
        self.follow_point(point_to_follow)

    def follow_point(self, pt):
        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - self.pose[2]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        distance = np.hypot(dx, dy)

        # self.get_logger().info(f"Angle difference: {np.degrees(angle_diff)}")
        self.get_logger().info(f"Idx: {self.traj_index}, Dist: {distance:.2f}, Angle: {angle_diff:.2f}")

        # Parameters
        max_lin_vel = 0.03  # m/s
        max_ang_vel = 0.05  # rad/s
        angle_gain = 0.1   # angular proportional gain
        slowdown_angle_thresh = 1.0  # rad
        min_lin_vel = 0.0

        # Adjust linear speed based on angle error
        speed_scale = max(0.0, 1.0 - abs(angle_diff) / slowdown_angle_thresh)
        cmd = Twist()
        cmd.linear.x = max(min_lin_vel, max_lin_vel * speed_scale) if distance > 0.05 else 0.0
        cmd.angular.z = np.clip(angle_gain * angle_diff, -max_ang_vel, max_ang_vel)

        if abs(angle_diff) > np.pi / 6:
            cmd.linear.x = 0.0

        self.pub_cmd.publish(cmd)
    
    def pure_pursuit(self):
        if self.spline is None or self.pose is None:
            return

        # Find the lookahead point
        lookahead_dist = 0.3
        x, y, theta = self.pose
        found = False
        for i in range(self.traj_index, self.spline.shape[1]):
            dx = self.spline[0, i] - x
            dy = self.spline[1, i] - y
            dist = np.hypot(dx, dy)
            if dist >= lookahead_dist:
                point_to_follow = np.array([self.spline[0, i], self.spline[1, i]])
                self.traj_index = i
                found = True
                break
        if not found:
            self.stop()
            return
        
        desired_theta = np.arctan2(point_to_follow[1], point_to_follow[0])
        angle_error = desired_theta - self.pose[2]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))


        # Transform to robot frame
        local_x = np.cos(-theta) * (point_to_follow[0] - x) - np.sin(-theta) * (point_to_follow[1] - y)
        local_y = np.sin(-theta) * (point_to_follow[0] - x) + np.cos(-theta) * (point_to_follow[1] - y)

        curvature = 2 * local_y / (lookahead_dist ** 2 + 1e-6)
        linear_velocity = 0.01
        angular_velocity = linear_velocity * curvature * 0.3

        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = np.clip(angular_velocity, -0.1, 0.1)

        if abs(angle_error) > np.deg2rad(30):
            cmd.linear.x = 0.0

        self.pub_cmd.publish(cmd)
    
    def publish_edt_as_grid(self, edt_array, resolution, origin, frame_id="base_footprint"):
        # Normalize EDT to [0, 100] (RViz expects this)
        max_dist = np.max(edt_array)
        if max_dist == 0:
            norm = np.zeros_like(edt_array, dtype=np.int8)
        else:
            norm = (1.0 - edt_array / max_dist) * 100  # Inverted: close = dark
            norm = np.clip(norm, 0, 100).astype(np.int8)

        # Convert to OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.info.resolution = resolution
        msg.info.width = edt_array.shape[1]
        msg.info.height = edt_array.shape[0]
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = norm.flatten().tolist()  # Row-major
        self.edt_pub.publish(msg)

    def stop(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BSOHFCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
