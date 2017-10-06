#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
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

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */

        /*@param x_in Initial state
      * @param P_in Initial state covariance
      * @param F_in Transition matrix
      * @param H_in Measurement matrix
      * @param R_in Measurement covariance matrix
      * @param Q_in Process covariance matrix
       */
//      ekf_.Init()

        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 0, 0, 0, 0; // px py vx vy

        // covariance
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

        ekf_.F_ = MatrixXd(4, 4); // updated later
        ekf_.Q_ = MatrixXd(4, 4); // updated later
        ekf_.H_ = H_laser_; // set later
        ekf_.R_ = R_laser_; // set later

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            double radius = measurement_pack.raw_measurements_(0);
            double angle = measurement_pack.raw_measurements_(1);
            ekf_.x_(0) = radius * cos(angle);
            ekf_.x_(1) = radius * sin(angle);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
        }
        cout << "initialized with:\nx_ = " << ekf_.x_.transpose() << endl;
        cout << "P_ = \n" << ekf_.P_ << endl;
        cout << endl;

        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double noise_ax = 9, noise_ay = 9;
    double delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // microseconds to seconds
    previous_timestamp_ = measurement_pack.timestamp_;

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


    ekf_.Predict();

//    cout << "predict:\n";
//    cout << "x_ = " << ekf_.x_.transpose() << endl;
//    cout << "P_ = \n" << ekf_.P_ << endl;
    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
//        cout << "Radar update: " ;
//        cout << measurement_pack.raw_measurements_.transpose() << endl;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
//        cout << "Laser update: " ;
//        cout << measurement_pack.raw_measurements_.transpose() << endl;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
//    cout << "update:\n";
//    cout << "x_ = " << ekf_.x_.transpose() << endl;
//    cout << "P_ = \n" << ekf_.P_ << endl;
//    cout << endl;
}
