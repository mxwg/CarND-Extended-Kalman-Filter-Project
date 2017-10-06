#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::endl;
using std::cout;

void KalmanFilter::Init(MatrixXd P_initialCovariance) {
    x_ = VectorXd(4);
    x_.setZero();
    P_ = P_initialCovariance;
    F_ = MatrixXd(4, 4);
    H_ = MatrixXd(2, 4);
    R_ = MatrixXd(2, 2);
    Q_ = MatrixXd(4, 4);
    I_ = Eigen::MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict() {
    // predict state
    x_ = F_ * x_; // + u

    // update covariance
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    Eigen::VectorXd z_pred = H_ * x_;
    Eigen::VectorXd y = z - z_pred;

    updateWithKalmanGain(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    Eigen::VectorXd y = z - h_radar();
    y(1) = atan2(sin(y(1)), cos(y(1))); // normalize angle

    updateWithKalmanGain(y);
}

void KalmanFilter::updateWithKalmanGain(Eigen::VectorXd &y) {
    Eigen::MatrixXd K = calculateKalmanGain();
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;
}

MatrixXd KalmanFilter::calculateKalmanGain() const {
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;

    return K;
}

VectorXd KalmanFilter::h_radar() const {
    // get current state
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    // map state to radar measurement
    double rho = sqrt(px * px + py * py);
    double theta = atan2(py, px);
    double rhoDot = (px * vx + py * vy) / rho;

    Eigen::VectorXd h_radar(3);
    h_radar << rho, theta, rhoDot;

    return h_radar;
}

void KalmanFilter::printState() const {
    cout << "x_ = " << x_.transpose() << endl;
    cout << "P_ = \n" << P_ << endl;
}
