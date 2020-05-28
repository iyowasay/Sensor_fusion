#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
// 0.097 0.085 0.4517 0.44
//
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  use_radar = true;
  use_lidar = true;
  previous_timestamp_ = 0;

  n_x = 4; // px,py,vx,vy
  n_lidar = 2;
  n_radar = 3;
  // initializing matrices
  R_laser_ = MatrixXd(n_lidar, n_lidar);
  R_radar_ = MatrixXd(n_radar, n_radar);
  H_laser_ = MatrixXd(n_lidar, n_x);
  Hj_ = MatrixXd(n_radar, n_x);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  noise_ax = 9.0;
  noise_ay = 9.0;
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  MatrixXd F = MatrixXd::Identity(n_x,n_x);
  MatrixXd Q = MatrixXd(n_x,n_x);
  VectorXd x = VectorXd::Zero(n_x);
  MatrixXd P = MatrixXd::Identity(n_x,n_x);
  P(0,0) = 1;
  P(1,1) = 1;
  P(2,2) = 1;
  P(3,3) = 1;
  ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, Q);


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  //measured data 
  VectorXd signals = measurement_pack.raw_measurements_;

  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      ekf_.Normalization(signals(1));

      ekf_.x_(0) = signals(0)*cos(signals(1)); // rho * cos(phi)
      ekf_.x_(1) = -(signals(0)*sin(signals(1))); // rho * sin(phi)
      ekf_.x_(2) = signals(2)*cos(signals(1)); //rho_dot
      ekf_.x_(3) = -(signals(2)*sin(signals(1)));

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_lidar) {
      // TODO: Initialize state.
      ekf_.x_(0) = signals(0); // px
      ekf_.x_(1) = signals(1); // py
      ekf_.x_(2) = 0; 
      ekf_.x_(3) = 0;

    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Q_ << pow(dt,4)/4*noise_ax, 0, pow(dt,3)/2*noise_ax, 0,
              0, pow(dt,4)/4*noise_ay, 0, pow(dt,3)/2*noise_ay,
              pow(dt,3)/2*noise_ax, 0, dt*dt*noise_ax, 0,
              0, pow(dt,3)/2*noise_ay, 0, dt*dt*noise_ay;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  ekf_.Predict();
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar) {
    // TODO: Radar updates (non linear EKF)
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    cout << "Update (Radar)" << endl;
    ekf_.UpdateEKF(signals, Hj_);

  } else if(use_lidar){
    // TODO: Laser updates
    cout << "Update (Lidar)" << endl;
    ekf_.Update(signals);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
