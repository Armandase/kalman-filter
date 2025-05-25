from filterpy.kalman import KalmanFilter as FP_KalmanFilter
import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO, VARIANCE_GPS
from utils import update_pos

class KakalmanFilter(FP_KalmanFilter):
    def __init__(self, true_pos, acceleration, speed, direction):
        super().__init__(dim_x=6, dim_z=6)
        self.F = np.eye(6)  # Matrice de transition (A)
        self.F[0, 3] = DELTA_T
        self.F[1, 4] = DELTA_T 
        self.F[2, 5] = DELTA_T  
        self.H = np.eye(6)  # Matrice d'observation
        self.Q = np.diag([       # Process noise covariance Q (9x9)
            # VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS,       # position x, y, z
            VARIANCE_GYRO, VARIANCE_GYRO, VARIANCE_GYRO,    # velocity x, y, z
            VARIANCE_ACCEL, VARIANCE_ACCEL, VARIANCE_ACCEL,    # velocity x, y, z
        ])
        self.R = np.diag([
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE,    # velocity x, y, z
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE  # acceleration x, y, z
        ])
        # self.R = np.eye(6) * 0.1
        self.P = np.eye(6) * 0.1
        self.z = np.zeros(6)
        self.x = np.concatenate((direction, acceleration))
        self.pos = np.array(true_pos) # position
        self.speed = float(speed[0]) # speed
        # self.speed /= 3.6  # Convert speed from km/h to m/s

    def update_position(self, speed, delta_t=DELTA_T):
        if not isinstance(speed, list) or len(speed) != 1:
            raise ValueError("Speed should be a list with one element.")

        if speed[0] != 0:
            self.speed = float(speed[0])
            # self.speed /= 3.6 

        accel = self.x[3:]
        direction = self.x[:3]
        print(f"Direction: {direction}, Acceleration: {accel}, Speed: {self.speed}")

        self.pos = update_pos(self.pos, acceleration=accel, direction=direction, delta_t=delta_t, speed=self.speed)
        return self.pos