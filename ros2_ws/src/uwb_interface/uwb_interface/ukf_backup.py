import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class UWB_UKF:
    def __init__(self, anchor_positions, dt=0.1):

        self.anchors = np.array(anchor_positions)
        self.dt = dt # 10 hz

        # State: [x, y, vx, vy]
        self.points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=0)

        self.ukf = UKF(dim_x=4, dim_z=4, dt=self.dt,
                       fx=self.fx,
                       hx=self.hx,
                       points=self.points)

        # Initial state estimate ([x, y, vx, vy])
        self.ukf.x = np.array([0, 0, 0, 0])

        # Covariances
        self.ukf.P *= 0.5  # Initial uncertainty
        self.ukf.R = np.diag([0.05, 0.05, 0.05, 0.05])  # Measurement noise (increase if its jumpy/too reactive)
        # self.ukf.R = np.diag([0.01, 0.01, 0.01, 0.01])
        self.ukf.Q = np.diag([0.001, 0.001, 0.1, 0.1])   # Process noise (increase if its too slow to follow)

        # self.ukf.Q = 

    def fx(self, x, dt):
        """State transition function for constant velocity model"""
        x_pos, y_pos, vx, vy = x
        return np.array([
            x_pos + vx * dt,
            y_pos + vy * dt,
            vx,
            vy
        ])

    def hx(self, x):
        """Measurement function: distance to each anchor"""
        x_pos, y_pos = x[0], x[1]
        return np.array([
            np.sqrt((x_pos - ax)**2 + (y_pos - ay)**2)
            for ax, ay in self.anchors
        ])

    def update(self, distances, dt=None):
        """Update UKF with new distance measurements"""
        if dt:
            self.ukf.dt = dt
        self.ukf.predict()
        self.ukf.update(np.array(distances))
        return self.ukf.x[:2]  # return x, y estimate

    def get_state(self):
        return self.ukf.x.copy()
