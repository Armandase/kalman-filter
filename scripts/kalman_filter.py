import numpy as np
from constants import DELTA_T, ACCEL_NOISE, GYRO_NOISE, GPS_NOISE, VARIANCE_ACCEL, VARIANCE_GYRO

class KalmanFilter():
    def __init__(self, direction, acceleration, speed, true_pos):
        print(f"Direction : {direction},\nacceleration : {acceleration},\nspeed : {speed},\ntrue_pos: {true_pos}")
        
        # estimated state vector
        self.estim_x = np.zeros(6)
        # Put the velocity in the state vector
        self.init_state_vector(direction, speed, true_pos)
        
        # previous estimated state vector
        self.prev_estim_x = np.zeros(6)
        
        # Kalman gain
        self.K = None
        
         # covariance matrix
        self.P = np.eye(6) * 0.1
        
        # previous covariance matrix
        self.prev_P = np.eye(6) * 0.1
        
        # observation matrix
        self.H = np.eye(6)
        
        # measurement noise covariance matrix
        self.R = np.eye(6) * 0.1
        
        # measurement vector
        self.Z = np.zeros(6)
        
        # identity matrix
        self.I = np.eye(6)
        
        #state transition matrix
        self.A = np.eye(6)
        self.A[0:3, 3:6] = np.eye(3) * DELTA_T

        # Control input vector
        self.U = np.array(acceleration)
        
        # COntrol input matrix
        self.B = np.array([
            [0.5 * DELTA_T ** 2, 0, 0],
            [0, 0.5 * DELTA_T ** 2, 0],
            [0, 0, 0.5 * DELTA_T ** 2],
            [DELTA_T, 0, 0],
            [0, DELTA_T, 0],
            [0, 0, DELTA_T]
        ])

        # Process noise covariance matrix
        sigma_pos = 0.5 * (DELTA_T ** 2) * ACCEL_NOISE
        sigma_vel = DELTA_T * ACCEL_NOISE
        
        # Calculating variance for position and velocity
        var_pos = sigma_pos ** 2
        var_vel = sigma_vel ** 2

        # Process noise covariance matrix
        self.Q = np.diag([
            var_pos, var_pos, var_pos,
            var_vel, var_vel, var_vel
        ])


    def measurement_update(self, direction, speed, true_pos):
        """
        Update the measurement vector with the current acceleration and direction
        Here it is each 3 seconds when we receive a new GPS position
        """

        # Update the measurement vector
        self.Z[0:3] = np.array(true_pos)
        self.Z[3:6] = self.calculate_velocity(speed, direction)

        # Measurement update
        self.K = self.prev_P @ self.H.T @ np.linalg.inv(self.H @ self.prev_P @ self.H.T + self.R)
        
        # Update the state estimate
        self.prev_estim_x = self.estim_x.copy()
        self.estim_x = self.prev_estim_x + self.K @ (self.Z - self.H @ self.prev_estim_x)
        
        # Update the covariance matrix
        self.prev_P = self.P.copy()
        self.P = (self.I - self.K @ self.H) @ self.prev_P


    def time_update(self, acceleration):
        """ Update the state vector and covariance matrix based on the model"""
        
        # Update the control input vector each time step
        self.U = np.array(acceleration)
        
        # Project the state ahead
        self.estim_x = self.A @ self.prev_estim_x + self.B @ self.U
        
        # Project the error covariance ahead
        self.P = self.A @ self.prev_P @ self.A.T + self.Q
        
        # Return the estimated state vector for the program to send the next position
        return self.estim_x[0:3]
    
    
    def init_state_vector(self, direction, speed, true_pos):
        """ Initialize the state vector with the given true position and velocity"""
        
        self.estim_x[0:3] = np.array(true_pos)
        self.estim_x[3:6] = self.calculate_velocity(speed, direction)
    
    
    def calculate_velocity(self, speed, direction):
        """ Calculate the velocity based on the current state vector """
        
        direction = np.array(direction)
        norm = np.linalg.norm(direction)
        if norm != 0:
            direction = direction / norm
        velocity = speed * direction
        return velocity