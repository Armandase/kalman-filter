# pragma once

#include <Eigen/Dense>
#include <iostream>

# define DT 0.01
# define ACCEL_NOISE 10e-3
# define GYRO_NOISE 10e-2
# define GPS_NOISE 10e-1
# define VARIANCE_ACCEL 10e-6
# define VARIANCE_GYRO 10e-4
# define VARIANCE_GPS 10e-2

using namespace Eigen;

MatrixXd rotationMatrixFromEuler(double roll, double pitch, double yaw);
VectorXd computeVelocity(const VectorXd &accel, const VectorXd &angles, const VectorXd &velocity, double dt);
std::string vectorToResponse(const VectorXd &vec);