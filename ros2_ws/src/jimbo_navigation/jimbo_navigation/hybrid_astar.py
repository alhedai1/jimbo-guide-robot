import numpy as np
import heapq
import math

class HybridAStarPlanner:
    def __init__(self, occ_grid, resolution, origin, logger, robot_radius=0.3):
        self.grid = occ_grid
        self.res = resolution
        self.origin = origin
        self.height, self.width = None, None
        self.robot_radius = robot_radius

        self.steering_angles = [-0.6, -0.3, 0.0, 0.3, 0.6]  # radians
        self.step_size = 0.5  # meters
        self.angle_res = np.deg2rad(30)
        self.num_angles = int(2 * np.pi / self.angle_res)

        self.logger = logger

        self.logger.info(f"Initialized Hybrid A* Planner")

    def update_grid(self, new_grid, resolution, origin):
        self.grid = new_grid
        self.res = resolution
        self.origin = origin
        self.height, self.width = self.grid.shape

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.res)
        gy = int((y - self.origin[1]) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.res + self.origin[0]
        y = gy * self.res + self.origin[1]
        return x, y

    # def is_valid(self, x, y):
    #     gx, gy = self.world_to_grid(x, y)
    #     if 0 <= gx < self.width and 0 <= gy < self.height:
    #         return self.grid[gy, gx] == 0
    #     self.logger.info("Point is invalid!")
    #     return False
    def is_valid(self, x, y):
        gx, gy = self.world_to_grid(x, y)

        radius_cells = int(self.robot_radius / self.res)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.grid[ny, nx] != 0:
                        return False
                else:
                    return False  # outside map is invalid
        return True

    def heuristic(self, x, y, goal_x, goal_y):
        return math.hypot(goal_x - x, goal_y - y)

    def normalize_angle(self, angle):
        return (angle + 2 * np.pi) % (2 * np.pi)

    def theta_to_index(self, theta):
        return int(self.normalize_angle(theta) / self.angle_res) % self.num_angles

    def simulate_motion(self, x, y, theta, delta, steps=1):
        # Kinematic model for diff drive (simplified)
        path = []
        for _ in range(steps):
            theta += delta
            x += self.step_size * math.cos(theta)
            y += self.step_size * math.sin(theta)
            path.append((x, y, theta))
        return path

    def plan(self, start, goal):
        # self.logger.info(f"Start planning...")
        sx, sy = start
        gx, gy = goal

        start_node = (sx, sy, 0.0)
        goal_node = (gx, gy)

        open_list = []
        closed_set = set()
        cost_so_far = {}

        def node_key(x, y, theta):
            gx, gy = self.world_to_grid(x, y)
            theta_idx = self.theta_to_index(theta)
            return (gx, gy, theta_idx)

        heapq.heappush(open_list, (0.0, 0.0, start_node, None))
        came_from = {}

        while open_list:
            # self.logger.info(f"open list length: {len(open_list)}")
            # self.logger.info(f"closed set length: {len(closed_set)}")
            _, cost, (x, y, theta), parent = heapq.heappop(open_list)
            key = node_key(x, y, theta)

            if key in closed_set:
                continue
            closed_set.add(key)
            came_from[key] = (parent, (x, y, theta))

            if self.heuristic(x, y, gx, gy) < self.step_size:
                # Reached goal
                # self.logger.info(f"Reached goal!")
                path = [(x, y, theta)]
                while parent is not None:
                    # self.logger.info(f"Parent is not None")
                    parent_key = node_key(*parent)
                    parent, _ = came_from[parent_key]
                    path.append(parent)
                path.reverse()
                _ = path.pop(0)
                return [(x, y) for x, y, theta in path]

            for delta in self.steering_angles:
                traj = self.simulate_motion(x, y, theta, delta)
                x_n, y_n, theta_n = traj[-1]

                if not self.is_valid(x_n, y_n):
                    # self.logger.info(f"Point is invalid!")
                    continue

                new_cost = cost + self.step_size
                next_key = node_key(x_n, y_n, theta_n)

                if next_key not in cost_so_far or new_cost < cost_so_far[next_key]:
                    cost_so_far[next_key] = new_cost
                    priority = new_cost + self.heuristic(x_n, y_n, gx, gy)
                    heapq.heappush(open_list, (priority, new_cost, (x_n, y_n, theta_n), (x, y, theta)))


        return None  # Failed to find path
