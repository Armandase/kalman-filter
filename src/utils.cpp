#include "../inc/main.h"

MatrixXd rotationMatrixFromEuler(double roll, double pitch, double yaw){
    MatrixXd Rx(3, 3);

    Rx << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);
    
    MatrixXd Ry(3, 3);
    Ry << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);
    
    MatrixXd Rz(3, 3);
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;
    
    MatrixXd R = Rx * Ry * Rz; // Combine rotations
    return R;
}

VectorXd computeVelocity(const VectorXd &accel, const VectorXd &angles, const VectorXd &velocity, double dt=DT) {
    MatrixXd R = rotationMatrixFromEuler(angles(0), angles(1), angles(2));

    VectorXd accelWorld = R * accel; // Rotate acceleration to world frame
    VectorXd newVelocity = velocity + accelWorld * dt; // Update velocity

    return newVelocity;
}


std::string vectorToResponse(const VectorXd &vec) {
    std::string response;
    for (int i = 0; i < vec.size(); ++i) {
        response += std::to_string(vec(i));
        if (i < vec.size() - 1) {
            response += " ";
        }
    }
    return response;
}