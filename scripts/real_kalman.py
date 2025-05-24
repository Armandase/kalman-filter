from filterpy.kalman import KalmanFilter as FP_KalmanFilter
import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO

class KakalmanFilter(FP_KalmanFilter):
    def __init__(self, true_pos, acceleration, speed, direction):
        super().__init__(dim_x=9, dim_z=6)
        self.F = np.eye(9)  # Matrice de transition
        self.F[0, 3] = DELTA_T  # px depends on vx
        self.F[1, 4] = DELTA_T  # py depends on vy
        self.F[2, 5] = DELTA_T  # pz depends on vz
        self.F[3, 6] = DELTA_T  # vx depends on ax
        self.F[4, 7] = DELTA_T  # vy depends on ay
        self.F[5, 8] = DELTA_T
        self.H = np.zeros((6, 9))  # Matrice d'observation
        for i in range(6):
            self.H[i, i + 3] = 1
        # self.Q = np.diag([GPS_NOISE] * 3 + [GYRO_NOISE] * 3 + [ACCEL_NOISE] * 3)
        self.Q = np.diag([       # Process noise covariance Q (9x9)
            GPS_NOISE, GPS_NOISE, GPS_NOISE,       # position x, y, z
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE,    # velocity x, y, z
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE  # acceleration x, y, z
        ])
        # self.R = np.diag([ACCEL_NOISE] * 3 + [GYRO_NOISE] * 3)
        self.R = np.diag([
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE,   # accelerometer noise (σ=0.001²)
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE    # gyroscope/direction noise (σ=0.01²)
        ])
        velocity = self.calculate_velocity(speed, direction)
        self.x = np.concatenate((true_pos, velocity, acceleration))
        self.P = np.eye(9)

        self.pos = np.array(true_pos) # position
        self.velocity = velocity # velocity
        self.speed = float(speed[0]) # speed

    def update_position(self, velocity, acceleration, delta_t=DELTA_T):
        # self.velocity = [speed * d for d in direction]
        self.velocity = np.array(velocity)
        acceleration = np.array(acceleration)
        self.pos = self.pos + self.velocity * delta_t + 0.5 * acceleration * (np.array(delta_t)**2)
        return self.pos
    
    def calculate_velocity(self, speed, direction):
        if not isinstance(speed, list) or len(speed) != 1:
            raise ValueError("Speed should be a list with one element.")

        if speed[0] != 0:
            self.speed = float(speed[0])
        velocity = self.speed * np.array(direction)
        return velocity