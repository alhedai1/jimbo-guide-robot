import math
from enum import Enum
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf_transformations import euler_from_quaternion
import rclpy

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')

        # robot parameter
        self.max_speed = 0.1  # [m/s]
        self.min_speed = -0.05  # [m/s]
        self.max_yaw_rate = 1.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.02  # [m/ss]
        self.max_delta_yaw_rate = 1.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.001  # [m/s]
        self.yaw_rate_resolution = 0.01 * math.pi / 180.0  # [rad/s]
        self.dt = 0.05  # [s] Time tick for motion prediction
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.5  # [m] for collision check

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
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def goal_callback(self, msg: PoseStamped):
        self.target = np.array([msg.pose.position.x, msg.pose.position.y])

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = np.array([pos.x, pos.y, yaw])
        self.state = np.array([pos.x, pos.y, yaw, v, w])

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

                obstacles.append((x, y))
        if not self.ob:
            self.ob = obstacles
    
    def control_loop(self):
        if self.state is None or self.target is None or self.occ_grid is None:
            return
        
        if not self.tracking:
            start = self.state
            goal = self.target
            distance = np.linalg.norm(goal - start[:2])
            self.get_logger().info(f'start: {start[:2]}, goal: {goal}, distance: {distance:.3f}')
            self.tracking = True

        # while True:
        u, trajectory = self.dwa_control(self.state, goal)
        self.get_logger(f"Best_u: {u} | Best Traj: {trajectory}")
        cmd = Twist()
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]
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
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
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

    # def make_ob(self):
    #     self.occupany_grid.data

# def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
#     plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#               head_length=width, head_width=width)
#     plt.plot(x, y)


# def plot_robot(x, y, yaw, config):  # pragma: no cover
#     if config.robot_type == RobotType.rectangle:
#         outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
#                              (config.robot_length / 2), -config.robot_length / 2,
#                              -config.robot_length / 2],
#                             [config.robot_width / 2, config.robot_width / 2,
#                              - config.robot_width / 2, -config.robot_width / 2,
#                              config.robot_width / 2]])
#         Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
#                          [-math.sin(yaw), math.cos(yaw)]])
#         outline = (outline.T.dot(Rot1)).T
#         outline[0, :] += x
#         outline[1, :] += y
#         plt.plot(np.array(outline[0, :]).flatten(),
#                  np.array(outline[1, :]).flatten(), "-k")
#     elif config.robot_type == RobotType.circle:
#         circle = plt.Circle((x, y), config.robot_radius, color="b")
#         plt.gcf().gca().add_artist(circle)
#         out_x, out_y = (np.array([x, y]) +
#                         np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
#         plt.plot([x, out_x], [y, out_y], "-k")


# def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
#     print(__file__ + " start!!")
#     # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
#     x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
#     # goal position [x(m), y(m)]
#     goal = np.array([gx, gy])

#     # input [forward speed, yaw_rate]

#     config.robot_type = robot_type
#     trajectory = np.array(x)
#     ob = config.ob
#     while True:
#         u, predicted_trajectory = dwa_control(x, config, goal, ob)
#         x = motion(x, u, config.dt)  # simulate robot
#         trajectory = np.vstack((trajectory, x))  # store state history

#         if show_animation:
#             plt.cla()
#             # for stopping simulation with the esc key.
#             plt.gcf().canvas.mpl_connect(
#                 'key_release_event',
#                 lambda event: [exit(0) if event.key == 'escape' else None])
#             plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
#             plt.plot(x[0], x[1], "xr")
#             plt.plot(goal[0], goal[1], "xb")
#             plt.plot(ob[:, 0], ob[:, 1], "ok")
#             plot_robot(x[0], x[1], x[2], config)
#             plot_arrow(x[0], x[1], x[2])
#             plt.axis("equal")
#             plt.grid(True)
#             plt.pause(0.0001)

#         # check reaching goal
#         dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
#         if dist_to_goal <= config.robot_radius:
#             print("Goal!!")
#             break

#     print("Done")
#     if show_animation:
#         plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
#         plt.pause(0.0001)
#         plt.show()


# if __name__ == '__main__':
#     # main(robot_type=RobotType.rectangle)
#     main(robot_type=RobotType.circle)