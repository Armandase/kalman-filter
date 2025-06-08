from filterpy.kalman import KalmanFilter as FP_KalmanFilter
import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO, VARIANCE_GPS
from utils_v2 import compute_velocity

class KakalmanFilter(FP_KalmanFilter):
    def __init__(self, true_pos, acceleration, speed, direction):
        super().__init__(dim_x=6, dim_z=6)
        self.F = np.eye(6)  # Matrice de transition (A)
        self.F[0, 3] = DELTA_T
        self.F[1, 4] = DELTA_T 
        self.F[2, 5] = DELTA_T  

        self.H = np.zeros((6, 6))  # Matrice d'observation
        self.H[0, 0] = 1  # x position
        self.H[1, 1] = 1  # y position
        self.H[2, 2] = 1 # z position
        # self.H[3, 3] = 1
        # self.H[4, 4] = 1
        # self.H[5, 5] = 1

        # self.H = np.eye(6)  # Matrice d'observation
        
        self.Q = np.diag([       # Process noise covariance
            [DELTA_T**2 / 2, 0, 0, DELTA_T**2 / 2, 0, 0],  # position x, y, z
            [0, DELTA_T**2 / 2, 0, 0, DELTA_T**2 / 2, 0],
            [0, 0, DELTA_T**2 / 2, 0, 0, DELTA_T**2 / 2],
            [0, 0, 0, DELTA_T, 0, 0],  # velocity x, y, z
            [0, 0, 0, 0, DELTA_T, 0],
            [0, 0, 0, 0, 0, DELTA_T]
        ])
        variance_v = GYRO_NOISE**2 + ACCEL_NOISE**2 * DELTA_T
        noise_pos = np.diag([VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS, 0, 0, 0])  # GPS noise covariance
        noise_accel = np.diag([VARIANCE_ACCEL * DELTA_T * DELTA_T / 2, 
                                VARIANCE_ACCEL * DELTA_T * DELTA_T / 2, 
                                VARIANCE_ACCEL * DELTA_T * DELTA_T / 2, 
                                VARIANCE_ACCEL * DELTA_T, 
                                VARIANCE_ACCEL * DELTA_T, 
                                VARIANCE_ACCEL * DELTA_T])
        noise_vel = np.diag([0, 0, 0, 
                              variance_v, variance_v, variance_v])
        noise_vel[0, 3] = DELTA_T * variance_v
        noise_vel[1, 4] = DELTA_T * variance_v
        noise_vel[2, 5] = DELTA_T * variance_v
        
        noise_density = noise_pos + noise_accel + noise_vel
        self.Q = self.Q @ noise_density

        process_noise_cov = self.F @ self.Q @ self.F.T
        self.Q = self.integrate(process_noise_cov, start=0, end=DELTA_T)
        self.R = np.diag([VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS, VARIANCE_ACCEL, VARIANCE_ACCEL, VARIANCE_ACCEL])  # Measurement noise covariance
        # self.R = np.eye(9)
        self.P = np.eye(6) * 0.1
        self.B_default = np.array([[DELTA_T**2 / 2, 0, 0],
        # self.B = np.array([[DELTA_T**2 / 2, 0, 0],
                     [0, DELTA_T**2 / 2, 0],
                     [0, 0, DELTA_T**2 / 2],
                     [DELTA_T, 0, 0],
                     [0, DELTA_T, 0],
                     [0, 0, DELTA_T]])
        self.z = np.zeros(6)
        velocity = compute_velocity(euler_angles=direction, delta_t=DELTA_T, velocity=np.zeros(3), acceleration=acceleration)
        print("Velocity: ", velocity)
        # self.x = np.concatenate((true_pos, [0, 0, 0]))
        self.x = np.concatenate((true_pos, velocity))
        self.pos = np.array(true_pos) # position
        self.speed = float(speed[0]) # speed
        # self.speed /= 3.6  # Convert speed from km/h to m/s

    def update_position(self, speed, delta_t=DELTA_T):
        if not isinstance(speed, list) or len(speed) != 1:
            raise ValueError("Speed should be a list with one element.")

        if speed[0] != 0:
            self.speed = float(speed[0])
            # self.speed /= 3.6 

        pos = self.x[:3]
        velocity = self.x[3:6]
        # self.pos = pos + velocity * delta_t + 0.5 * np.array([0, 0, -9.81]) * (delta_t ** 2)
        self.pos = pos + velocity * delta_t + 0.5 * (delta_t ** 2)
        return self.pos
    
    def integrate(self, m = None, start=0, end=DELTA_T):
        if m is None:
            m = self.F
        n = 10000
        dx = (end - start) / n
        
        res = np.zeros_like(m)
        for i in range(n):
            res += m * dx
        return res