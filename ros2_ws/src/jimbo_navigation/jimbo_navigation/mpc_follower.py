import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Bool, Header
from tf_transformations import euler_from_quaternion
import numpy as np
import cvxpy as cp
import math

class MPCFollower(Node):
    def __init__(self):
        super().__init__('mpc_follower')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.pose = None  # [x, y, theta]
        self.path = None  # np.ndarray of shape (2, N)

        # TODO: add subscriptions for odometry and path
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, 'bso_hfc_path', self.path_callback, 10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])
    
    def path_callback(self, msg):
        poses = msg.poses
        if not poses:
            return

        xy = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in poses])
        self.path = xy.T

        self.get_logger().info(f"Received path of shape: {self.path.shape}")

    def control_loop(self):
        if self.pose is None or self.path is None:
            return

        x0 = np.array(self.pose)  # current state [x, y, theta]
        dt = 0.1
        H = 3

        # 1. Extract reference path segment
        ref = self.get_reference_segment(x0, H, dt)
        if ref is None:
            self.stop()
            return

        # 2. Build MPC problem
        x = cp.Variable((3, H + 1))  # [x, y, theta]
        u = cp.Variable((2, H))     # [v, omega]

        cost = 0
        constraints = [x[:, 0] == x0]

        for t in range(H):
            # Dynamics
            theta = x[2, t]
            v = u[0, t]
            w = u[1, t]

            theta0 = x0[2]
            cos_theta = np.cos(theta0)
            sin_theta = np.sin(theta0)

            next_x = x[0, t] + dt * cos_theta * u[0, t]
            next_y = x[1, t] + dt * sin_theta * u[0, t]
            next_theta = x[2, t] + dt * w

            constraints += [
                x[0, t + 1] == next_x,
                x[1, t + 1] == next_y,
                x[2, t + 1] == next_theta,
                cp.abs(v) <= 0.01,
                cp.abs(w) <= 0.06,
            ]

            # Tracking cost
            cost += cp.sum_squares(x[0:2, t] - ref[:, t])
            cost += 0.1 * cp.square(v) + 0.1 * cp.square(w)

        # 3. Solve
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        if u.value is not None:
            cmd = Twist()
            cmd.linear.x = float(u.value[0, 0])
            cmd.angular.z = float(u.value[1, 0])
            self.pub.publish(cmd)
        else:
            self.get_logger().warn("MPC failed to solve")
            self.stop()

    def get_reference_segment(self, x0, H, dt):
        """ Return a (2, H) reference segment (x, y) from self.path near x0 """
        if self.path.shape[1] < H:
            return None

        dists = np.linalg.norm(self.path.T - x0[:2], axis=1)
        start = np.argmin(dists)
        self.get_logger().info(f"Start: {start}")

        end = start + H
        if end >= self.path.shape[1]:
            end = self.path.shape[1] - 1
            start = max(0, end - H)

        ref_segment = self.path[:, start:end]
        if ref_segment.shape[1] < H:
            # pad with last point
            last = ref_segment[:, -1:]
            pad = np.repeat(last, H - ref_segment.shape[1], axis=1)
            ref_segment = np.concatenate((ref_segment, pad), axis=1)

        # self.get_logger().info(f"Ref: {ref_segment}")

        return ref_segment

    def stop(self):
        cmd = Twist()
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = MPCFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
