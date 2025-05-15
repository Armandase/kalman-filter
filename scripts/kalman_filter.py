import numpy as np

class KalmanFilter():
    def __init__(self):
        self.A = np.eye(3) # state transition matrix
        self.H = np.eye(3) # used to map the measurement space to the state space
        self.Q = np.eye(3) # state noise covariance matrix
        self.R = np.eye(3) # measurement noise covariance matrix

        self.P = np.eye(3) # error covariance matrix
        self.pred_P = np.eye(3) # prediction of the error covariance matrix

        self.estim_x = np.zeros((3, 1))
        self.pred_x = np.zeros((3, 1))

    def predict(self):
        prediction = self.A @ self.estim_x
        self.pred_P = self.A @ self.P @ self.A.T + self.Q
        return prediction,

    def update(self, z):
        """
        Update the state with the measurement z.
        z is the measurement vector.
        """
        K = self.pred_P @ self.H.T @ np.linalg.inv(self.H @ self.pred_P @ self.H.T + self.R)

        self.estim_x = self.pred_x + K @ (z + self.H @ self.pred_x)
        self.P = self.pred_P - self.K @ self.H @ self.pred_P