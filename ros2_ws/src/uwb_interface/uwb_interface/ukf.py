import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class UWB_UKF:
    def __init__(self, anchor_positions, dt=0.1):
        self.anchors = np.array(anchor_positions)
        self.dt = dt

        self.points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=0)
        self.ukf = UKF(dim_x=4, dim_z=len(anchor_positions), dt=self.dt,
                       fx=self.fx, hx=self.hx_fixed, points=self.points)

        self.ukf.x = np.array([0, 0, 0, 0])
        self.ukf.P *= 0.5
        self.ukf.R = np.diag([0.05] * len(anchor_positions))
        self.ukf.Q = np.diag([0.001, 0.001, 0.1, 0.1])

        self._current_anchors = self.anchors

    def fx(self, x, dt):
        x_pos, y_pos, vx, vy = x
        return np.array([x_pos + vx * dt, y_pos + vy * dt, vx, vy])

    def hx_fixed(self, x):
        x_pos, y_pos = x[0], x[1]
        return np.array([
            np.sqrt((x_pos - ax)**2 + (y_pos - ay)**2)
            for ax, ay in self._current_anchors
        ])

    def update(self, anchors, distances, dt=None):
        if dt:
            self.ukf.dt = dt
        self._current_anchors = np.array(anchors)

        # Adjust R to match the number of measurements
        self.ukf.R = np.diag([0.05] * len(distances))

        self.ukf.predict()
        self.ukf.update(np.array(distances), hx=self.hx_fixed)
        return self.ukf.x[:2]

    def get_state(self):
        return self.ukf.x.copy()
