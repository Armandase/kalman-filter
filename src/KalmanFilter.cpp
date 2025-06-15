# include "../inc/KalmanFilter.h"
# include "../inc/main.h"

KalmanFilter::KalmanFilter(int dimZ, int dimX) {
    this->x = VectorXd::Zero(dimX); // State vector 
    this->F = MatrixXd::Identity(dimX, dimX); // State transition matrix
    this->P = MatrixXd::Identity(dimX, dimX); // State covariance matrix
    this->H = MatrixXd::Zero(dimZ, dimX); // Measurement matrix
    this->R = MatrixXd::Identity(dimZ, dimZ); // Measurement noise covariance
    this->Q = MatrixXd::Identity(dimX, dimX); // Process noise covariance
    this->B = MatrixXd::Zero(dimX, dimZ); // Control input matrix
    this->I = MatrixXd::Identity(dimX, dimX); // Identity matrix

    this->H(0, 0) = 1; // x position
    this->H(1, 1) = 1; // y position
    this->H(2, 2) = 1; // z position

    this->dimZ = dimZ; // Dimension of measurement vector
    this->dimX = dimX; // Dimension of state vector

    this->initialized = false; // Flag to check if matrices are initialized
}

void KalmanFilter::initMatrices(const VectorXd &pos, const VectorXd &accel, const VectorXd &dir, double speed) {
    std::cout << "Initializing KalmanFilter matrices..." << std::endl;
    VectorXd defaultVelocity(3); // Default velocity vector initialized to zero
    defaultVelocity(0) = speed / 3.6; // Set the x component to speed
    // Compute the velocity based on speed and direction
    VectorXd velocity(3);
    velocity = computeVelocity(accel, dir, defaultVelocity, DT);

    // Initialize state vector with position, velocity, and acceleration
    this->x << pos(0), pos(1), pos(2), velocity(0), velocity(1), velocity(2);

    // Initialize state transition matrix F based on speed and direction
    this->F = MatrixXd::Identity(dimX, dimX); // State transition matrix
    this->F(0, 3) = DT; // x position update
    this->F(1, 4) = DT; // y position update
    this->F(2, 5) = DT; // z position update

    this->Q = MatrixXd::Zero(dimX, dimX); // Process noise covariance
    double q_pos = 0.5 * (DT * DT); // Position uncertainty from acceleration
    this->Q << 
        q_pos * q_pos * VARIANCE_ACCEL, 0, 0, q_pos * DT * VARIANCE_ACCEL, 0, 0,
        0, q_pos * q_pos * VARIANCE_ACCEL, 0, 0, q_pos * DT * VARIANCE_ACCEL, 0,
        0, 0, q_pos * q_pos * VARIANCE_ACCEL, 0, 0, q_pos * DT * VARIANCE_ACCEL,
        q_pos * DT * VARIANCE_ACCEL, 0, 0, DT * DT * VARIANCE_ACCEL, 0, 0,
        0, q_pos * DT * VARIANCE_ACCEL, 0, 0, DT * DT * VARIANCE_ACCEL, 0,
        0, 0, q_pos * DT * VARIANCE_ACCEL, 0, 0, DT * DT * VARIANCE_ACCEL;

    // DiagonalMatrix<double, 3> m(3, 8, 6);
    this->R = DiagonalMatrix<double, 3>(VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS); // Measurement noise covariance

    double varianceVelocity = VARIANCE_GYRO + VARIANCE_ACCEL * (DT * DT);
    this->P = DiagonalMatrix<double, 6>(VARIANCE_GPS, VARIANCE_GPS, VARIANCE_GPS, varianceVelocity, varianceVelocity, varianceVelocity); // State covariance matrix
    this->P = this->P * 10;
    double step = 0.5 * (DT * DT);
    this->B << 
        step, 0, 0,
        0, step, 0,
        0, 0, step,
        DT, 0, 0,
        0, DT, 0,
        0, 0, DT; // Control input matrix
    this->initialized = true; // Set initialized flag to true


    // std::cout << "F:\n" << this->F << std::endl;
    // std::cout << "P:\n" << this->P << std::endl;
    // std::cout << "H:\n" << this->H << std::endl;
    // std::cout << "R:\n" << this->R << std::endl;
    // std::cout << "Q:\n" << this->Q << std::endl;
    // std::cout << "B:\n" << this->B << std::endl;
    // std::cout << "x:\n" << this->x << std::endl;
}

void KalmanFilter::predict() {
    if (!this->initialized) {
        std::cerr << "KalmanFilter matrices not initialized. Call initMatrices first." << std::endl;
        return;
    }

    // Predict the next state
    this->x = this->F * this->x; // State prediction
    this->P = this->F * this->P * this->F.transpose() + this->Q; // Covariance prediction
}

void KalmanFilter::predict(const VectorXd &u) {
    if (!this->initialized) {
        std::cerr << "KalmanFilter matrices not initialized. Call initMatrices first." << std::endl;
        return;
    }
    // Predict the next state with control input
    // std::cout << "Current F: " << this->F << std::endl;
    // std::cout << "Current state: " << this->x << std::endl;
    // std::cout << "Current B: " << this->B << std::endl;
    // std::cout << "Control input: " << u << std::endl;
    this->x = this->F * this->x + this->B * u; // State prediction with control input
    this->P = this->F * this->P * this->F.transpose() + this->Q; // Covariance prediction
    // std::cout << "Predicted state: " << this->x << std::endl;
    // std::cout << "Predicted covariance: \n" << this->P << std::endl;
    // exit(0); // Exit after prediction for debugging purposes
}

void KalmanFilter::update(const VectorXd &z) {
    if (!this->initialized) {
        std::cerr << "KalmanFilter matrices not initialized. Call initMatrices first." << std::endl;
        return;
    }
    // Update the state with measurement
    MatrixXd PHT = this->P * this->H.transpose(); // Precompute P * H^T
    MatrixXd S = this->H * PHT + this->R; // Innovation covariance

    MatrixXd K = PHT * S.inverse(); // Kalman gain

    this->x = this->x + K * (z - this->H * this->x); // State update
    this->P = (this->I - K * this->H) * this->P; // Covariance update
}

VectorXd KalmanFilter::getPos() const {
    // Return the position part of the state vector
    return this->x.head(3); // Assuming the first three elements are the position
}

VectorXd KalmanFilter::getVelocity() const {
    // Return the velocity part of the state vector
    return this->x.segment(3, 3); // Assuming the next three elements are the velocity
}

bool KalmanFilter::isInitialized() const {
    return this->initialized;
}