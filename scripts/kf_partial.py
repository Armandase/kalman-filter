import numpy as np

class KalmanFilter():
    def __init__(self, dt, inital_pos, initial_speed):
        self.dt = dt
        # vecteur d'état initial
        self.E = np.array([inital_pos, initial_speed]) # x, y, z, vx, vy, vz
        # matrice de transition
        # self.A = np.array([[1, 0, self.dt, 0],
        #                    [0, 1, 0, self.dt],
        #                    [0, 0, 1, 0],
        #                    [0, 0, 0, 1]])
        if isinstance(self.dt, list):
            if len(self.dt) == 1:
                self.dt = self.dt[0]
            else:
                raise Exception("To many delta time provided to Kalman filter")
        elif not isinstance(self.dt, (int, float)):
            raise Exception("Wrong type of detla time provided to Kalman filter")

        self.A = np.identity(4)
        self.A[0, -2] = self.dt
        self.A[1, -1] = self.dt
        # matrice d'observation (définit les mesures que l'on prend)
        self.H = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0]])
        # bruit relative par rapport a l'évolution de l'objet / du systeme
        self.Q = np.identity(4)
        # bruit relative aux mesures
        self.R = np.array([[1, 0],
                           [0, 1]])
        self.P = np.eye(self.A.shape[1]) # estimation de la covariance de l'erreur
    def predict(self):
        # prediction de la prochine pos
        self.E = np.dot(self.A, self.E)
        # calcul de la coraviance de l'erreur
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.E
    
    def update(self, Z):
        # calcul du gain de Kalman qui s'affine au fil des mesures et prediction
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # correction / innnovation
        self.E = np.round(self.E + (K @ (Z - (self.H @ self.E))))
        I = np.eye(self.H.shape[1])
        self.P = (I - K * self.H) * self.P

        # E = E + np.dot(K, (self.Z - H * E))
        # P = (I - K @ H) @ np.linalg.inv(P)