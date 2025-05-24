from filterpy.kalman import KalmanFilter as FP_KalmanFilter
import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO, VARIANCE_GPS
from utils import update_pos

class KakalmanFilter(FP_KalmanFilter):
    def __init__(self, true_pos, acceleration, speed, direction):
        super().__init__(dim_x=9, dim_z=6)
        self.F = np.eye(9)  # Matrice de transition (A)
        self.F[0, 3] = DELTA_T
        self.F[1, 4] = DELTA_T 
        self.F[2, 5] = DELTA_T  
        self.F[3, 6] = DELTA_T
        self.F[4, 7] = DELTA_T
        self.F[5, 8] = DELTA_T
        # self.H = np.eye(9)  # Matrice d'observation
        # self.H[0, 0] = 0
        # self.H[1, 1] = 0
        # self.H[2, 2] = 0
        self.H = np.zeros((6, 9))  # Matrice d'observation
        for i in range(6):
            self.H[i, i + 3] = 1
        self.Q = np.diag([       # Process noise covariance Q (9x9)
            GPS_NOISE, GPS_NOISE, GPS_NOISE,       # position x, y, z
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE,    # velocity x, y, z
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE  # acceleration x, y, z
        ])
        self.R = np.diag([
            # VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS,   # accelerometer noise (σ=0.001²)
            # VARIANCE_GYRO, VARIANCE_GYRO, VARIANCE_GYRO,   # accelerometer noise (σ=0.001²)
            # VARIANCE_ACCEL, VARIANCE_ACCEL, VARIANCE_ACCEL    # gyroscope/direction noise (σ=0.01²)
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE,    # velocity x, y, z
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE  # acceleration x, y, z
        ])
        self.x = np.concatenate((true_pos, direction, acceleration))
        self.P = np.diag([
            10, 10, 10,
            1, 1, 1,
            0.1, 0.1, 0.1
        ])

        self.pos = np.array(true_pos) # position
        self.speed = float(speed[0]) # speed
        self.speed /= 3.6  # Convert speed from km/h to m/s

    def update_position(self, speed, delta_t=DELTA_T):
        if not isinstance(speed, list) or len(speed) != 1:
            raise ValueError("Speed should be a list with one element.")

        if speed[0] != 0:
            self.speed = float(speed[0])
            self.speed /= 3.6

        accel = self.x[6:9]
        direction = self.x[3:6]

        self.pos = update_pos(self.pos, acceleration=accel, direction=direction, delta_t=delta_t, speed=self.speed)
        return self.pos