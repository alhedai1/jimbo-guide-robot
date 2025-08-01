import math
from enum import Enum
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf_transformations import euler_from_quaternion
import math
from jimbo_msgs.msg import MotorRPM

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')

        # # robot parameter
        # self.max_speed = 5.0  # [m/s]
        # self.min_speed = -0.5  # [m/s]
        # self.max_yaw_rate = 10.0 * math.pi / 180.0  # [rad/s]
        # self.max_accel = 2.0  # [m/ss]
        # self.max_delta_yaw_rate = 10.0 * math.pi / 180.0  # [rad/ss]
        # self.v_resolution = 0.01  # [m/s]
        # self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        # self.dt = 0.1  # [s] Time tick for motion prediction
        # self.predict_time = 3.0  # [s]
        # self.to_goal_cost_gain = 1.0
        # self.speed_cost_gain = 5.0
        # self.obstacle_cost_gain = 1.0
        # self.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked

        # # robot parameter
        self.max_speed = 10.0  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yaw_rate = 10.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_delta_yaw_rate = 10.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.5 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.3  # [m] for collision check

        # if robot_type == RobotType.rectangle
        # self.robot_width = 0.5  # [m] for collision check
        # self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        # self.ob = np.array([[-1, -1],

        self.ob = None

        self.pose = None
        self.state = None
        self.target = None
        self.occ_grid = None
        self.grid_res = None
        self.grid_origin = None
        self.tracking = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_target = self.create_subscription(PointStamped, '/uwb_filtered_position', self.uwb_callback, 10)
        self.sub_occupancy = self.create_subscription(OccupancyGrid, 'inflated_occupancy_grid', self.occupancy_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.rpm_sub = self.create_subscription(MotorRPM, 'motor_rpm', self.rpm_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'dwa_path', 10)

        self.timer = self.create_timer(0.1, self.control_loop) # 10 hz

    def goal_callback(self, msg: PoseStamped):
        self.target = np.array([msg.pose.position.x, msg.pose.position.y])

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        # self.get_logger().info(f"v: {v}, w: {w}")
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])
        self.state = np.array([pos.x, pos.y, yaw, v, w])
    
    def rpm_callback(self, msg: MotorRPM):
        return
        # self.get_logger().info(f"Left RPM: {msg.left_rpm}, Right RPM: {msg.right_rpm}")

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
    
    def occupancy_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        self.occ_grid = np.array(data, dtype=np.int8).reshape((height, width))
        self.grid_res = msg.info.resolution
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        # self.edt = self.compute_edt(self.occ_grid, self.grid_res)

        obstacles = []
        for idx, value in enumerate(data):
            if value > 50:
                map_x = idx % width
                map_y = idx // width

                x = self.grid_origin[0] + (map_x + 0.5) * self.grid_res
                y = self.grid_origin[1] + (map_y + 0.5) * self.grid_res

                obstacles.append([x,y])
        if self.ob is None:
            self.ob = np.array(obstacles)
            # for obs in self.ob:
            #     self.get_logger().info(f"obstacle: {np.round(obs, 2)}, distance: {np.round(np.linalg.norm(obs), 2)}")
    
    def control_loop(self):
        # if self.state is not None:
        #     self.get_logger().info(f"State: {np.round(self.state, 3)}")

        if self.state is None or self.target is None or self.occ_grid is None:
            return

        if not self.tracking:
            start = self.state
            goal = self.target
            distance = np.linalg.norm(goal - start[:2])
            self.get_logger().info(f'start: {start}, goal: {goal}, distance: {distance:.3f}')
            self.tracking = True

        # while True:
        u, trajectory = self.dwa_control(self.state, self.target)
        self.publish_path(trajectory[:, :2])
        # self.get_logger().info(f"Best_u: [{u[0]:.2f}, {u[1]:.2f}] | Trajectory: {np.round(trajectory[-1], 2)}")
        # self.get_logger().info(f"state: {self.state}")
        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]
        self.get_logger().info(f"Sending cmd: x = {u[0]}, z = {u[1]}")
        self.pub_cmd.publish(cmd)

    def dwa_control(self, x, goal):
        dw = self.calc_dynamic_window(x)
        u, trajectory = self.calc_control_and_trajectory(x, dw, goal)
        return u, trajectory


    def motion(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x):
        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        # self.get_logger().info(f"dw: {dw}, x[3]: {x[3]}")
        self.get_logger().info(f"current v: {x[3]}")
        return dw


    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory


    def calc_control_and_trajectory(self, x, dw, goal):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                # self.get_logger().info(f"max speed: {self.max_speed}, current speed: {trajectory[-1, 3]}")
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                # self.get_logger().info(f"speed cost: {speed_cost}")
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:

                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory):
        """
        calc obstacle cost inf: collision
        """
        ox = self.ob[:, 0]
        oy = self.ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if np.array(r <= self.robot_radius).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def publish_path(self, traj):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'  # Adjust if you're in another frame

        for i in range(traj.shape[0]):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = traj[i, 0]
            pose.pose.position.y = traj[i, 1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # no rotation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def stop(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=None)
    node = DWAController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()