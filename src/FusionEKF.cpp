#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    // measurement matrix
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        initializeEkfOnFirstMeasurement(measurement_pack);
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    // update time
    double delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // microseconds to seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    updateTimeDependentMatrices(delta_t);

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        setRadarMatrices();
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        setLaserMatrices();
        ekf_.Update(measurement_pack.raw_measurements_);
    }
}

void FusionEKF::initializeEkfOnFirstMeasurement(const MeasurementPackage &measurement_pack) {
    // initial covariance
    MatrixXd P = MatrixXd(4, 4);
    P <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    ekf_.Init(P);

    // initialize state according to measurement type
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state
        double radius = measurement_pack.raw_measurements_(0);
        double angle = measurement_pack.raw_measurements_(1);
        ekf_.x_(0) = radius * cos(angle);
        ekf_.x_(1) = radius * sin(angle);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state
        ekf_.x_(0) = measurement_pack.raw_measurements_(0);
        ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // initialize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    cout << "EKF initialized with: " << endl;
    ekf_.printState();

    // done initializing, no need to predict or update
    is_initialized_ = true;
}

void FusionEKF::setRadarMatrices() {
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
}

void FusionEKF::setLaserMatrices() {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
}

void FusionEKF::updateTimeDependentMatrices(double delta_t) {
    double noise_ax = 9, noise_ay = 9;

    ekf_.F_ <<
            1, 0, delta_t, 0,
            0, 1, 0, delta_t,
            0, 0, 1, 0,
            0, 0, 0, 1;
    double dt_2 = delta_t * delta_t;
    double dt_3 = dt_2 * delta_t;
    double dt_4 = dt_3 * delta_t;

    ekf_.Q_ <<
            dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
}
