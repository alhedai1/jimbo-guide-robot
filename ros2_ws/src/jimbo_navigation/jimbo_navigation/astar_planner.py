import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, occ_grid, resolution, origin):
        """
        occ_grid: 2D numpy array (0=free, 1=obstacle)
        resolution: meters per cell
        origin: (x0, y0) bottom-left of the grid in world coordinates
        """
        self.grid = occ_grid
        self.res = resolution
        self.origin = origin
        self.height, self.width = occ_grid.shape
        self.motion = [  # 8 directions
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.res)
        gy = int((y - self.origin[1]) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.res + self.origin[0]
        y = gy * self.res + self.origin[1]
        return x, y

    def heuristic(self, a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])  # Euclidean

    def is_valid(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y, x] == 0

    def plan(self, start_world, goal_world):
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        open_list = []
        heapq.heappush(open_list, (0 + self.heuristic((sx, sy), (gx, gy)), 0, (sx, sy), None))
        came_from = {}
        cost_so_far = { (sx, sy): 0 }

        while open_list:
            _, cost, current, parent = heapq.heappop(open_list)

            if current == (gx, gy):
                path = [(gx, gy)]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [self.grid_to_world(x, y) for x, y in path]

            if current in came_from:
                continue
            came_from[current] = parent

            for dx, dy in self.motion:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.is_valid(nx, ny):
                    continue
                new_cost = cost + np.hypot(dx, dy)
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + self.heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_list, (priority, new_cost, (nx, ny), current))
        return None
