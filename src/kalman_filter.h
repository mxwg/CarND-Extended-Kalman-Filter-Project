#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    // identity matrix
    Eigen::MatrixXd I_;

    /**
     * Constructor
     */
    KalmanFilter() = default;

    /**
     * Destructor
     */
    virtual ~KalmanFilter() = default;

    /**
     * Init Initializes Kalman filter
     * @param P_initialCovariance Initial state covariance
     */
    void Init(Eigen::MatrixXd P_initialCovariance);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);

    /**
     * Prints the current state of the EKF (x and P).
     */
    void printState() const;

private:
    /// Update x_ and P_ using y
    void updateWithKalmanGain(Eigen::VectorXd &y);

    /// Calculate the Kalman gain K
    Eigen::MatrixXd calculateKalmanGain() const;

    /// Calculate h'(x) for radar predictions
    Eigen::VectorXd h_radar() const;
};

#endif /* KALMAN_FILTER_H_ */
