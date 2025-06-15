# pragma once

#include <Eigen/Dense>
#include <iostream>


using namespace Eigen;

class KalmanFilter
{
    public:
        KalmanFilter(int dimZ, int dimX);

        void initMatrices(const VectorXd &pos, const VectorXd &accel, const VectorXd &dir, double speed);

        void predict();
        void predict(const VectorXd &u);
        void update(const VectorXd &z);

        VectorXd getPos() const;
        VectorXd getVelocity() const;
        bool isInitialized() const;
    private:
        int dimZ; // Dimension of measurement vector
        int dimX; // Dimension of state vector

        VectorXd x; // State vector
        MatrixXd P; // State covariance matrix
        MatrixXd F; // State transition matrix
        MatrixXd Q; // Process noise covariance matrix
        MatrixXd H; // Measurement matrix
        MatrixXd R; // Measurement noise covariance matrix
        MatrixXd B; // Control input matrix
        MatrixXd I; // Identity matrix

        bool initialized; // Flag to check if matrices are initialized
};  