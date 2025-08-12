import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class UWB_UKF:
    def __init__(self, tag_positions, dt=0.1):
        self.tags = np.array(tag_positions)
        self.dt = dt

        self.points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=0)
        self.ukf = UKF(dim_x=4, dim_z=len(tag_positions), dt=self.dt,
                       fx=self.fx, hx=self.hx, points=self.points)

        # initial state
        self.ukf.x = np.array([0, 0, 0, 0])
        
        self.ukf.P *= 0.5 # initial uncertainty
        self.ukf.R = np.diag([0.05] * len(tag_positions)) # measurement noise
        self.ukf.Q = np.diag([0.001, 0.001, 0.1, 0.1]) # process noise [x, y, vx, vy]

        self.valid_tags = self.tags

    def fx(self, x, dt):
        x_pos, y_pos, vx, vy = x
        return np.array([x_pos + vx * dt, y_pos + vy * dt, vx, vy])

    def hx(self, x):
        x_pos, y_pos = x[0], x[1]
        return np.array([
            np.sqrt((x_pos - ax)**2 + (y_pos - ay)**2)
            for ax, ay in self.valid_tags
        ])

    def update(self, tags, distances, dt=None):
        if dt:
            self.ukf.dt = dt
        self.valid_tags = np.array(tags)

        # Adjust R to match the number of measurements
        self.ukf.R = np.diag([0.05] * len(distances))
        self.ukf._dim_z = len(self.valid_tags)

        self.ukf.predict()
        self.ukf.update(np.array(distances), hx=self.hx)
        return self.ukf.x[:2]

    def get_state(self):
        return self.ukf.x.copy()
