import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO

class KalmanFilter():
    def __init__(self, direction, acceleration, speed, true_pos):
        # print(f"Direction : {direction},\nacceleration : {acceleration},\nspeed : {speed},\ntrue_pos: {true_pos}")
        self.A = np.eye(9) # state transition matrix
        self.A[0, 3] = DELTA_T  # px depends on vx
        self.A[1, 4] = DELTA_T  # py depends on vy
        self.A[2, 5] = DELTA_T  # pz depends on vz
        
        self.A[3, 6] = DELTA_T  # vx depends on ax
        self.A[4, 7] = DELTA_T  # vy depends on ay
        self.A[5, 8] = DELTA_T  # vz depends on az

        self.H = np.zeros((6, 9)) # used to map the measurement space to the state space
        for i in range(6):
            self.H[i, i + 3] = 1
        self.Q = np.diag([       # Process noise covariance Q (9x9)
            GPS_NOISE, GPS_NOISE, GPS_NOISE,       # position x, y, z
            GYRO_NOISE, GYRO_NOISE, GYRO_NOISE,    # velocity x, y, z
            ACCEL_NOISE, ACCEL_NOISE, ACCEL_NOISE  # acceleration x, y, z
        ])
        # self.R = np.diag([
        #     0.01,        # speed variance (σ=0.1²)
        #     1e-6, 1e-6, 1e-6,   # accelerometer noise (σ=0.001²)
        #     1e-4, 1e-4, 1e-4    # gyroscope/direction noise (σ=0.01²)
        # ])
        self.R = np.diag([
            VARIANCE_ACCEL, VARIANCE_ACCEL, VARIANCE_ACCEL,   # accelerometer noise (σ=0.001²)
            VARIANCE_GYRO, VARIANCE_GYRO, VARIANCE_GYRO    # gyroscope/direction noise (σ=0.01²)
        ])

        self.P = np.eye(9) # error covariance matrix
        self.pred_P = np.eye(3) # prediction of the error covariance matrix

        velocity = self.calculate_velocity(speed, direction)
        # print(velocity)
        self.estim_x = np.concatenate((true_pos, velocity, acceleration)) # [px, py, pz, vx, vy, vz, accelx, accely, accelz]
        self.pred_x = np.concatenate((true_pos, velocity, acceleration)) # [px, py, pz, vx, vy, vz, accelx, accely, accelz]
        # print("x : ", self.estim_x)
        self.pos = np.array(true_pos) # position
        self.velocity = None # velocity

    def predict(self):
        self.pred_x = self.A @ self.estim_x
        self.pred_P = self.A @ self.P @ self.A.T + self.Q
        return self.pred_x

    def update(self, z):
        """
        Update the state with the measurement z.
        z is the measurement vector.
        """
        K = self.pred_P @ self.H.T @ np.linalg.inv(self.H @ self.pred_P @ self.H.T + self.R)

        self.estim_x = self.pred_x + K @ (z - self.H @ self.pred_x)
        self.P = self.pred_P - K @ self.H @ self.pred_P

    def update_position(self, velocity, acceleration, delta_t=DELTA_T):
        # self.velocity = [speed * d for d in direction]
        self.velocity = np.array(velocity)
        acceleration = np.array(acceleration)
        self.pos = self.pos + self.velocity * delta_t + 0.5 * acceleration * (np.array(delta_t)**2)
        return self.pos


    def calculate_velocity(self, speed, direction):
        """
        Calculate the velocity vector given speed and direction.

        Parameters:
        - speed: A scalar value representing the speed.
        - direction: A NumPy array representing the direction.

        Returns:
        - velocity: A vector representing the velocity.
        """

        velocity = speed * np.array(direction)
        return velocity