import numpy as np

from constants import DELTA_T, VARIANCE_ACCEL, VARIANCE_GYRO, VARIANCE_GPS, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE
from utils_v2 import compute_velocity

class KalmanFilter():
    def __init__(self, true_pos, acceleration, speed, direction):
        
        # State transition matrix (constant velocity model)
        self.F = np.eye(6)
        self.F[0, 3] = DELTA_T  # x position depends on x velocity
        self.F[1, 4] = DELTA_T  # y position depends on y velocity
        self.F[2, 5] = DELTA_T  # z position depends on z velocity

        # Observation matrix - observe position and velocity
        # self.H = np.eye(6)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1  # Observe x position
        self.H[1, 1] = 1  # Observe y position
        self.H[2, 2] = 1  # Observe z position
        
        # Simplified process noise matrix
        # Use constant acceleration model for process noise
        q_pos = 0.5 * DELTA_T**2  # Position uncertainty from acceleration
        q_vel = DELTA_T             # Velocity uncertainty
        # q_pos = GPS_NOISE * DELTA_T**2  # Position uncertainty from acceleration
        # q_vel = GYRO_NOISE ** 2 + ACCEL_NOISE ** 2 * DELTA_T              # Velocity uncertainty
        
        self.Q = np.array([
            [q_pos**2 * VARIANCE_ACCEL, 0, 0, q_pos * q_vel * VARIANCE_ACCEL, 0, 0],
            [0, q_pos**2 * VARIANCE_ACCEL, 0, 0, q_pos * q_vel * VARIANCE_ACCEL, 0],
            [0, 0, q_pos**2 * VARIANCE_ACCEL, 0, 0, q_pos * q_vel * VARIANCE_ACCEL],
            [q_pos * q_vel * VARIANCE_ACCEL, 0, 0, q_vel**2 * VARIANCE_ACCEL, 0, 0],
            [0, q_pos * q_vel * VARIANCE_ACCEL, 0, 0, q_vel**2 * VARIANCE_ACCEL, 0],
            [0, 0, q_pos * q_vel * VARIANCE_ACCEL, 0, 0, q_vel**2 * VARIANCE_ACCEL]
        ])
        
        # Measurement noise covariance
        variance_v = VARIANCE_GYRO + VARIANCE_ACCEL * DELTA_T**2
        self.R = np.diag([VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS, 
                         variance_v, variance_v, variance_v])
        
        # Initial covariance - start with higher uncertainty
        self.P = np.diag([VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS,
                         variance_v, variance_v, variance_v]) * 10
        
        # Control matrix for acceleration input
        self.B = np.array([[0.5 * DELTA_T**2, 0, 0],
                          [0, 0.5 * DELTA_T**2, 0],
                          [0, 0, 0.5 * DELTA_T**2],
                          [DELTA_T, 0, 0],
                          [0, DELTA_T, 0],
                          [0, 0, DELTA_T]])
        
        # Initialize speed and velocity
        self.speed = float(speed[0]) / 3.6  # Convert km/h to m/s
        
        # Compute initial velocity from direction and speed
        velocity = compute_velocity(euler_angles=direction, delta_t=DELTA_T, 
                                  velocity=np.array([self.speed, 0, 0]), 
                                  acceleration=acceleration)
        
        # Initial state: [x, y, z, vx, vy, vz]
        self.x = np.concatenate((np.array(true_pos), velocity))
    
        self.estim_x = self.x.copy()
        self.pred_P = self.P.copy()

    def predict(self, U=None):
        if U is None:
            U = np.zeros((3,))

        self.x = self.F @ self.estim_x + self.B @ U
        self.pred_P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, z):
        """
        Update the state with the measurement z.
        z is the measurement vector.
        """
        K = self.pred_P @ self.H.T @ np.linalg.inv(self.H @ self.pred_P @ self.H.T + self.R)

        self.estim_x = self.x + K @ (z - self.H @ self.x)
        self.P = self.pred_P - K @ self.H @ self.pred_P
        # I = np.eye(self.P.shape[0])
        # self.P = (I - K @ self.H) @ self.pred_P